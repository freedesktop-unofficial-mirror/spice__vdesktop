#include "hw/hw.h"
#include "audio.h"
#define AUDIO_CAP "vd_interface_audio"
#include "audio_int.h"
#include "qemu-timer.h"
#include "interface.h"

//#define WAVE_CAPTURE
#ifdef WAVE_CAPTURE

#include <fcntl.h>

#define WAVE_BUF_SIZE (1024 * 1024 * 20)

typedef struct __attribute__ ((__packed__)) ChunkHeader {
    uint32_t id;
    uint32_t size;
} ChunkHeader;

typedef struct __attribute__ ((__packed__)) FormatInfo {
    uint16_t compression_code;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t average_bytes_per_second;
    uint16_t block_align;
    uint16_t bits_per_sample;
    //uint16_t extra_format_bytes;
    //uint8_t extra[0];
} FormatInfo;

static uint8_t* wave_buf = NULL;
static uint8_t* wave_now = NULL;
static uint8_t* wave_end = NULL;
static int wave_blocked = 0;

static void write_all(int fd, uint8_t* data, uint32_t size)
{
    while (size) {
        int n = write(fd, data, size);
        if (n == -1) {
            if (errno != EINTR) {
                printf("%s: failed\n", __FUNCTION__);
                exit(-1);
            }
        } else {
            data += n;
            size -= n;
        }
    }
}

static void write_wave(void)
{
    static uint32_t file_id = 0;
    char file_name[100];
    ChunkHeader header;
    FormatInfo format;

    if (wave_buf == wave_now) {
        return;
    }

    sprintf(file_name, "/tmp/vdi_%u.wav", ++file_id);
    int fd = open(file_name, O_CREAT| O_TRUNC | O_WRONLY, 0666);
    if (fd == -1) {
        printf("%s: open file %s failed\n", __FUNCTION__, file_name);
        return;
    }

    memcpy((char *)&header.id, "RIFF", 4);
    header.size = 4;
    write_all(fd, (uint8_t *)&header, sizeof(header));
    write_all(fd, (uint8_t *)"WAVE", 4);
    
    memcpy((char *)&header.id, "fmt ", 4);
    header.size = sizeof(format);
    write_all(fd, (uint8_t *)&header, sizeof(header));
    
    format.compression_code = 1;
    format.num_channels = 2;
    format.sample_rate = 44100;
    format.average_bytes_per_second = format.sample_rate * 4;
    format.block_align = 4;
    format.bits_per_sample = 16;
    write_all(fd, (uint8_t *)&format, sizeof(format));
    
    memcpy((char *)&header.id, "data", 4);
    header.size = wave_now - wave_buf;
    write_all(fd, (uint8_t *)&header, sizeof(header));
    write_all(fd, wave_buf, header.size);
    close(fd);
}

static void init_wave(void)
{
    if (!wave_buf) {
        wave_buf = malloc(WAVE_BUF_SIZE);
    }
    wave_now = wave_buf;
    wave_end = wave_buf + WAVE_BUF_SIZE;
}

static void start_wave(void)
{
    wave_blocked = 0;
    wave_now = wave_buf;
}

static void put_wave_data(uint8_t *data, uint32_t size)
{
    if (wave_blocked || size > wave_end - wave_now) {
        wave_blocked = 1;
        return;
    }
    memcpy((void *)wave_now, (void *)data, size);
    wave_now += size;
}

static void end_wave(void)
{
    write_wave();
}

#endif

#define dprintf(format, ...) \
    printf("%s: " format "\n", __FUNCTION__, ## __VA_ARGS__ )

#define ASSERT(x) if (!(x)) {                           \
    printf("%s: ASSERT %s failed\n", __FUNCTION__, #x); \
    abort();                                           \
}

#define LINE_IN_SAMPLES 1024
#define LINE_OUT_SAMPLES 1024

typedef struct InterfaceVoiceOut {
    HWVoiceOut base;
    uint64_t prev_ticks;
} InterfaceVoiceOut;

typedef struct InterfaceVoiceIn {
    HWVoiceOut base;
    uint64_t prev_ticks;
    uint32_t samples[LINE_IN_SAMPLES];
} InterfaceVoiceIn;

static struct audio_option options[] = {
    {NULL, 0, NULL, NULL, NULL, 0}
};

typedef struct Interface_audio {
    PlaybackInterface *play_interface;
    PlaybackPlug *play_plug;
    uint32_t play_avail;
    uint32_t *play_frame;
    uint32_t *play_now;

    RecordInterface *record_interface;
    RecordPlug *record_plug;
    uint32_t silence[LINE_IN_SAMPLES];

    InterfaceVoiceOut *voic_out;
    HWVoiceIn *voic_in;
} Interface_audio;

static Interface_audio driver = {
    .play_interface = NULL,
    .play_plug = NULL,
    .play_avail = 0,
    .play_frame = NULL,
    .play_now = NULL,

    .record_interface = NULL,
    .record_plug = NULL,

    .voic_out = NULL,
    .voic_in = NULL,
};

static VDObjectRef interface_play_plug(PlaybackInterface *playback, PlaybackPlug* plug,
                                int *enabled)
{
    if (driver.play_plug) {
        dprintf("plug failed");
        return INVALID_VD_OBJECT_REF;
    }
    ASSERT(plug && playback == driver.play_interface && enabled);
    driver.play_plug = plug;
    *enabled = driver.voic_out ? driver.voic_out->base.enabled : 0;
    return (VDObjectRef)plug;
}

static void interface_play_unplug(PlaybackInterface *playback, VDObjectRef obj)
{
    if (!driver.play_plug || obj != (VDObjectRef)driver.play_plug) {
        dprintf("unplug failed");
        return;
    }
    ASSERT(playback == driver.play_interface);
    driver.play_plug = NULL;
    driver.play_avail = 0;
    driver.play_frame = NULL;
    driver.play_now = NULL;
}

static void regitser_playback(void)
{
    PlaybackInterface *interface = (PlaybackInterface *)qemu_mallocz(sizeof(*interface));
    static int playback_interface_id = 0;

    if (!interface) {
        printf("%s: malloc failed\n", __FUNCTION__);
        exit(-1);
    }
    interface->base.base_vertion = VM_INTERFACE_VERTION;
    interface->base.type = VD_INTERFACE_PLAYBACK;
    interface->base.id = playback_interface_id++;
    interface->base.description = "playback";
    interface->base.major_vertion = VD_INTERFACE_PLAYBACK_MAJOR;
    interface->base.minor_vertion = VD_INTERFACE_PLAYBACK_MINOR;

    interface->plug = interface_play_plug;
    interface->unplug = interface_play_unplug;

    driver.play_interface = interface;
    add_interface(&interface->base);
}

static VDObjectRef interface_record_plug(RecordInterface *recorder, RecordPlug* plug,
                                  int *enabled)
{
    ASSERT(!driver.record_plug && plug && recorder == driver.record_interface
           && enabled);
    driver.record_plug = plug;
    *enabled = driver.voic_in ? driver.voic_in->enabled : 0;
    return (VDObjectRef)plug;
}

static void interface_record_unplug(RecordInterface *recorder, VDObjectRef obj)
{
    ASSERT(driver.record_plug && recorder == driver.record_interface);
    driver.record_plug = NULL;
}

static void regitser_record(void)
{
    RecordInterface *interface = (RecordInterface *)qemu_mallocz(sizeof(*interface));
    static int record_interface_id = 0;

    if (!interface) {
        printf("%s: malloc failed\n", __FUNCTION__);
        exit(-1);
    }
    interface->base.base_vertion = VM_INTERFACE_VERTION;
    interface->base.type = VD_INTERFACE_RECORD;
    interface->base.id = record_interface_id++;
    interface->base.description = "record";
    interface->base.major_vertion = VD_INTERFACE_RECORD_MAJOR;
    interface->base.minor_vertion = VD_INTERFACE_RECORD_MINOR;

    interface->plug = interface_record_plug;
    interface->unplug = interface_record_unplug;

    driver.record_interface = interface;
    add_interface(&interface->base);
}

static void *interface_audio_init(void)
{
    dprintf("");
    if (VD_INTERFACE_PLAYBACK_FMT != VD_INTERFACE_AUDIO_FMT_S16 ||
        VD_INTERFACE_PLAYBACK_CHAN != 2 ||
        VD_INTERFACE_RECORD_FMT != VD_INTERFACE_AUDIO_FMT_S16 ||
        VD_INTERFACE_RECORD_CHAN != 2) {
        dprintf("bad dormat");
        exit(-1);
    }
    
    ASSERT(driver.play_interface == NULL && driver.record_interface == NULL);
    memset(driver.silence, 0, sizeof(driver.silence));
    regitser_playback();
    regitser_record();
    return &driver;
}

static void interface_audio_fini(void *opaque)
{
    dprintf("");

    if (driver.play_interface) {
        remove_interface(&driver.play_interface->base);
        ASSERT(!driver.play_plug);
        free(driver.play_interface);
        driver.play_interface = NULL;
    }

    if (driver.record_interface) {
        remove_interface(&driver.record_interface->base);
        ASSERT(!driver.record_plug);
        free(driver.record_interface);
        driver.record_interface = NULL;
    }
}

static const char *fmt_to_str(audfmt_e fmt)
{
    switch(fmt) {
    case AUD_FMT_U8:
        return "AUD_FMT_U8";
    case AUD_FMT_S8:
        return "AUD_FMT_S8";
    case AUD_FMT_U16:
        return "AUD_FMT_U16";
    case AUD_FMT_S16:
        return "AUD_FMT_S16";
    case AUD_FMT_U32:
        return "AUD_FMT_U32";
    case AUD_FMT_S32:
        return "AUD_FMT_S32";
    default:
        return "???";
    }
}

static int line_out_init(HWVoiceOut *hw, struct audsettings *as)
{
    InterfaceVoiceOut *voice_out = (InterfaceVoiceOut *)hw;
    struct audsettings settings;

    dprintf("freq %d channels %d format %s%s",
            as->freq, 
            as->nchannels, 
            fmt_to_str(as->fmt),
            as->endianness == AUDIO_HOST_ENDIANNESS ? " HOST_ENDIANNESS" : "" );


    settings.freq = VD_INTERFACE_PLAYBACK_FREQ; 
    settings.nchannels = VD_INTERFACE_PLAYBACK_CHAN;
    settings.fmt = AUD_FMT_S16;
    settings.endianness = AUDIO_HOST_ENDIANNESS;

    audio_pcm_init_info(&hw->info, &settings);
    hw->samples = LINE_OUT_SAMPLES;
    driver.voic_out = voice_out;
    return 0;
}

static void line_out_fini(HWVoiceOut *hw)
{
    driver.voic_out = NULL;
}

static uint64_t get_monotonic_time(void)
{
    struct timespec time_space;
    clock_gettime(CLOCK_MONOTONIC, &time_space);
    return time_space.tv_sec * 1000 * 1000 * 1000 + time_space.tv_nsec;
}

static int line_out_run(HWVoiceOut *hw)
{
    InterfaceVoiceOut *voice_out = (InterfaceVoiceOut *)hw;
    int rpos, live, decr;
    int samples;
    uint64_t now;
    uint64_t ticks;
    uint64_t bytes;

    if (!(live = audio_pcm_hw_get_live_out(hw))) {
        return 0;
    }

    now = get_monotonic_time();
    ticks = now - voice_out->prev_ticks;
    bytes = (ticks * hw->info.bytes_per_second) / (1000 * 1000 * 1000);

    voice_out->prev_ticks = now;

    decr = (bytes > INT_MAX) ? INT_MAX >> hw->info.shift :
                                        (bytes + (1 << (hw->info.shift - 1))) >> hw->info.shift;
    decr = audio_MIN(live, decr);

    samples = decr;
    rpos = hw->rpos;
    while (samples) {
        int left_till_end_samples = hw->samples - rpos;
        int len = audio_MIN(samples, left_till_end_samples);

        if (driver.play_plug && !driver.play_frame) {
            driver.play_plug->get_frame(driver.play_plug,
                                        &driver.play_frame,
                                        &driver.play_avail);
            driver.play_now = driver.play_frame;
        }
        if (driver.play_avail) {
            len = audio_MIN(len, driver.play_avail);
            hw->clip(driver.play_now, hw->mix_buf + rpos, len);
            if (!(driver.play_avail -= len)) {
                driver.play_plug->put_frame(driver.play_plug,
                                            driver.play_frame);
                driver.play_frame = driver.play_now = NULL;
            } else {
                driver.play_now += len;
            }
        }
        rpos = (rpos + len) % hw->samples;
        samples -= len;
    }
    hw->rpos = rpos;
    return decr;
}

static int line_out_write(SWVoiceOut *sw, void *buf, int len)
{
    return audio_pcm_sw_write(sw, buf, len);
}

static int line_out_ctl(HWVoiceOut *hw, int cmd, ...)
{
    InterfaceVoiceOut *voice_out = (InterfaceVoiceOut *)hw;

    switch (cmd) {
    case VOICE_ENABLE:
        voice_out->prev_ticks = get_monotonic_time();
        if (driver.play_plug) {
            driver.play_plug->start(driver.play_plug);
        }
        break;
    case VOICE_DISABLE:
        if (driver.play_plug) {
            if (driver.play_frame) {
                uint32_t *frame = driver.play_frame;
                memset(driver.play_now, 0, driver.play_avail << 2);
                driver.play_avail = 0;
                driver.play_now = driver.play_frame = NULL;
                driver.play_plug->put_frame(driver.play_plug, frame);
            }
            driver.play_plug->stop(driver.play_plug);
        }
        break;
    }
    return 0;
}

static int line_in_init(HWVoiceIn *hw, struct audsettings *as)
{
    struct audsettings settings;

    dprintf("");
#ifdef WAVE_CAPTURE
    init_wave();
#endif
    settings.freq = VD_INTERFACE_RECORD_FREQ; 
    settings.nchannels = VD_INTERFACE_RECORD_CHAN;
    if (VD_INTERFACE_PLAYBACK_FMT != VD_INTERFACE_AUDIO_FMT_S16) {
        dprintf("bad vd interface format");
        return -1;
    }
    settings.fmt = AUD_FMT_S16;
    settings.endianness = AUDIO_HOST_ENDIANNESS;

    audio_pcm_init_info(&hw->info, &settings);
    hw->samples = LINE_IN_SAMPLES;
    driver.voic_in = hw;
    return 0;
}

static void line_in_fini(HWVoiceIn *hw)
{
    driver.voic_in = NULL;
}

static int line_in_run(HWVoiceIn *hw)
{
    InterfaceVoiceIn *voice_in = (InterfaceVoiceIn *)hw;
    int num_samples;
    int ready;
    int len[2];
    uint64_t now;
    uint64_t ticks;
    uint64_t delta_samp;
    uint32_t *samples;

    if (!(num_samples = hw->samples - audio_pcm_hw_get_live_in(hw))) {
        return 0;
    }

    now = get_monotonic_time();
    ticks = now - voice_in->prev_ticks;
    voice_in->prev_ticks = now;
    delta_samp = (ticks * hw->info.bytes_per_second) / (1000 * 1000 * 1000);
    delta_samp = (delta_samp + (1 << (hw->info.shift - 1))) >> hw->info.shift;

    num_samples = audio_MIN(num_samples, delta_samp);

    if (driver.record_plug) {
        ready = driver.record_plug->read(driver.record_plug, num_samples, voice_in->samples);
        samples = voice_in->samples;
    } else {
        ready = num_samples;
        samples = driver.silence;
    }

    num_samples = audio_MIN(ready, num_samples);

    if (hw->wpos + num_samples > hw->samples) {
        len[0] = hw->samples - hw->wpos;
        len[1] = num_samples - len[0];
    } else {
        len[0] = num_samples;
        len[1] = 0;
    }

#ifdef WAVE_CAPTURE
    put_wave_data((uint8_t *)samples, len[0] * 4);
#endif
    hw->conv(hw->conv_buf + hw->wpos, samples, len[0], &nominal_volume);

    if (len[1]) {
#ifdef WAVE_CAPTURE
        put_wave_data((uint8_t *)(samples + len[0]), len[1] * 4);
#endif
        hw->conv(hw->conv_buf, samples + len[0], len[1],
                 &nominal_volume);
    }

    hw->wpos = (hw->wpos + num_samples) % hw->samples;

    return num_samples;
}

static int line_in_read(SWVoiceIn *sw, void *buf, int size)
{
    return audio_pcm_sw_read(sw, buf, size);
}

static int line_in_ctl(HWVoiceIn *hw, int cmd, ...)
{
    InterfaceVoiceIn *voice_in = (InterfaceVoiceIn *)hw;

    switch (cmd) {
    case VOICE_ENABLE:
#ifdef WAVE_CAPTURE
        start_wave();
#endif
        voice_in->prev_ticks = get_monotonic_time();
        if (driver.record_plug) {
            driver.record_plug->start(driver.record_plug);
        }
        break;
    case VOICE_DISABLE:
#ifdef WAVE_CAPTURE
        end_wave();
#endif
        if (driver.record_plug) {
            driver.record_plug->stop(driver.record_plug);
        }
        break;
    }
    return 0;
}


static struct audio_pcm_ops audio_callbacks = {
    line_out_init,
    line_out_fini,
    line_out_run,
    line_out_write,
    line_out_ctl,

    line_in_init,
    line_in_fini,
    line_in_run,
    line_in_read,
    line_in_ctl,
};

struct audio_driver interface_audio_driver = {
    INIT_FIELD (name = ) "vd_interface",
    INIT_FIELD (descr = ) "vd_interface audio driver",
    INIT_FIELD (options = ) options,
    INIT_FIELD (init = ) interface_audio_init,
    INIT_FIELD (fini = ) interface_audio_fini,
    INIT_FIELD (pcm_ops = ) &audio_callbacks,
    INIT_FIELD (can_be_default = ) 1,
    INIT_FIELD (max_voices_out = ) 1,
    INIT_FIELD (max_voices_in  = ) 1,
    INIT_FIELD (voice_size_out = ) sizeof (InterfaceVoiceOut),
    INIT_FIELD (voice_size_in  = ) sizeof (InterfaceVoiceIn),
};

