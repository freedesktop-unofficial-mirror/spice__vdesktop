#include "hw/hw.h"
#include "audio.h"
#define AUDIO_CAP "vd_interface_audio"
#include "audio_int.h"
#include "qemu-timer.h"
#include "interface.h"

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
    uint32_t record_avail;
    uint32_t *record_now;
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
    .record_avail = 0,
    .record_now = NULL,

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
    driver.record_avail = 0;
    driver.record_now = NULL;
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
    decr = (bytes > INT_MAX) ? INT_MAX >> hw->info.shift : bytes >> hw->info.shift;
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
    int live;
    int dead;
    int samples;
    int ready;
    int len[2];
    uint64_t now;
    uint64_t ticks;
    uint64_t bytes;


    live = audio_pcm_hw_get_live_in(hw);

    if (!(dead = hw->samples - live)) {
        return 0;
    }

    now = get_monotonic_time();
    ticks = now - voice_in->prev_ticks;
    voice_in->prev_ticks = now;
    bytes = (ticks * hw->info.bytes_per_second) / (1000 * 1000 * 1000);

    if (driver.record_avail) {
        ready = driver.record_avail;
    } else if (driver.record_plug) {
        driver.record_plug->advance(driver.record_plug,
                                    &driver.record_now,
                                    &driver.record_avail);
        ready = driver.record_avail;
    } else {
        ready = INT_MAX;
    }
    samples = audio_MIN(ready, dead);
    samples = audio_MIN(samples, bytes >> hw->info.shift);

    if (hw->wpos + samples > hw->samples) {
        len[0] = hw->samples - hw->wpos;
        len[1] = samples - len[0];
    } else {
        len[0] = samples;
        len[1] = 0;
    }

    if (driver.record_avail) {
        hw->conv(hw->conv_buf + hw->wpos, driver.record_now, len[0],
                 &nominal_volume);
        if (len[1]) {
            hw->conv(hw->conv_buf, driver.record_now + len[0], len[1],
                     &nominal_volume);
        }
        driver.record_now += samples;
        driver.record_avail -= samples;
    } else {
        hw->conv(hw->conv_buf + hw->wpos, driver.silence, len[0],
                 &nominal_volume);
        if (len[1]) {
            hw->conv(hw->conv_buf, driver.silence, len[1],
                     &nominal_volume);
        }
    }

    hw->wpos = (hw->wpos + samples) % hw->samples;

    return samples;
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
        voice_in->prev_ticks = get_monotonic_time();
        if (driver.record_plug) {
            driver.record_plug->start(driver.record_plug);
        }
        break;
    case VOICE_DISABLE:
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

