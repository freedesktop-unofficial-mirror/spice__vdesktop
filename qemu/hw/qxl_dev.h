#ifndef _H_QXL_DEV
#define _H_QXL_DEV

#include "ring.h"
#include "draw.h"

#ifdef __GNUC__
#ifdef __i386__
#define mb() __asm__ __volatile__ ("lock; addl $0,0(%%esp)": : :"memory")
#else
//mfence
#define mb() __asm__ __volatile__ ("lock; addl $0,0(%%rsp)": : :"memory")
#endif
#define ATTR_PACKED __attribute__ ((__packed__))
#else
#pragma pack(push)
#pragma pack(1)
#define ATTR_PACKED
#define mb() __asm {lock add [esp], 0}
#endif

#define REDHAT_PCI_VENDOR_ID 0x1b36
#define QXL_DEVICE_ID 0x01ff /* 0x100-0x11f reserved for spice, 0x1ff used for unstable work */
#define QXL_REVISION 0x01

#define QXL_ROM_MAGIC (*(UINT32*)"QXRO")
#define QXL_RAM_MAGIC (*(UINT32*)"QXRA")

enum {
    QXL_RAM_RANGE_INDEX,
    QXL_VRAM_RANGE_INDEX,
    QXL_ROM_RANGE_INDEX,
    QXL_IO_RANGE_INDEX,

    QXL_PCI_RANGES
};

enum {
    QXL_IO_NOTIFY_CMD,
    QXL_IO_NOTIFY_CURSOR,
    QXL_IO_UPDATE_AREA,
    QXL_IO_UPDATE_IRQ,
    QXL_IO_NOTIFY_OOM,
    QXL_IO_RESET,
    QXL_IO_SET_MODE,                  /* qxl-1 */
    QXL_IO_LOG,
    QXL_IO_MEMSLOT_ADD,
    QXL_IO_MEMSLOT_DEL,
    QXL_IO_DETACH_PRIMARY,
    QXL_IO_ATTACH_PRIMARY,
    QXL_IO_CREATE_PRIMARY,
    QXL_IO_DESTROY_PRIMARY,
    QXL_IO_DESTROY_SURFACE_WAIT,
    QXL_IO_DESTROY_ALL_SURFACES,
    QXL_IO_NOTIFY_SURFACES_OOM,

    QXL_IO_RANGE_SIZE
};

typedef struct ATTR_PACKED QXLRom {
    UINT32 magic;
    UINT32 id;
    UINT32 update_id;
    UINT32 compression_level;
    UINT32 log_level;
    uint32_t mode;                    /* qxl-1 */
    UINT32 modes_offset;
    UINT32 num_pages;
    uint32_t pages_offset;            /* qxl-1 */
    uint32_t draw_area_offset;        /* qxl-1 */
    uint32_t surface0_area_size;      /* qxl-1 name: draw_area_size */
    UINT32 ram_header_offset;
    UINT32 mm_clock;
    UINT32 n_surfaces;
    UINT64 flags;
    UINT8 slots_start;
    UINT8 slots_end;
    UINT8 slot_gen_bits;
    UINT8 slot_id_bits;
    UINT8 slot_generation;
    UINT8 padding[3];
} QXLRom;

typedef struct ATTR_PACKED QXLMode {
    UINT32 id;
    UINT32 x_res;
    UINT32 y_res;
    UINT32 bits;
    UINT32 stride;
    UINT32 x_mili;
    UINT32 y_mili;
    UINT32 orientation;
} QXLMode;

typedef struct ATTR_PACKED QXLModes {
    UINT32 n_modes;
    QXLMode modes[0];
} QXLModes;

typedef UINT64 PHYSICAL;
typedef UINT32 QXLFIXED; //fixed 28.4

enum QXLCmdType {
    QXL_CMD_NOP,
    QXL_CMD_DRAW,
    QXL_CMD_UPDATE,
    QXL_CMD_CURSOR,
    QXL_CMD_MESSAGE,
};

typedef struct ATTR_PACKED QXLCommand {
    PHYSICAL data;
    UINT32 type;
    UINT32 ped;
} QXLCommand;

typedef struct ATTR_PACKED QXLCommandExt {
    QXLCommand cmd;
    uint32_t group_id;
    uint32_t flags;
} QXLCommandExt;

typedef struct ATTR_PACKED QXLMemSlot {
    UINT64 mem_start;
    UINT64 mem_end;
} QXLMemSlot;

#define QXL_SURF_TYPE_PRIMARY 0

typedef struct ATTR_PACKED QXLSurfaceCreate {
    UINT32 width;
    UINT32 height;
    INT32 stride;
    UINT32 format;
    UINT32 position;
    UINT32 mouse_mode;
    UINT32 flags;
    UINT32 type;
    PHYSICAL mem;
} QXLSurfaceCreate;

RING_DECLARE(QXLCommandRing, QXLCommand, 32);
RING_DECLARE(QXLCursorRing, QXLCommand, 32);

RING_DECLARE(QXLReleaseRing, UINT64, 8);

#define QXL_LOG_BUF_SIZE 4096

#define QXL_INTERRUPT_DISPLAY (1 << 0)
#define QXL_INTERRUPT_CURSOR (1 << 1)

typedef struct ATTR_PACKED QXLRam {
    UINT32 magic;
    UINT32 int_pending;
    UINT32 int_mask;
    UINT8 log_buf[QXL_LOG_BUF_SIZE];   
    QXLCommandRing cmd_ring;
    QXLCursorRing cursor_ring;
    QXLReleaseRing release_ring;
    Rect update_area;
    UINT32 update_surface;
    QXLMemSlot mem_slot;
    QXLSurfaceCreate create_surface;
    UINT64 flags;
} QXLRam;

typedef union QXLReleaseInfo{
    UINT64 id;      // in
    UINT64 next;    // out
} QXLReleaseInfo;

typedef struct QXLReleaseInfoExt {
    QXLReleaseInfo *info;
    UINT32 group_id;
} QXLReleaseInfoExt;

typedef struct  ATTR_PACKED QXLDataChunk {
    UINT32 data_size;
    PHYSICAL prev_chunk;
    PHYSICAL next_chunk;
    UINT8 data[0];
} QXLDataChunk;

typedef struct ATTR_PACKED QXLMessage {
    QXLReleaseInfo release_info;
    UINT8 data[0];
} QXLMessage;

typedef struct ATTR_PACKED QXLUpdateCmd {
    QXLReleaseInfo release_info;
    Rect area;
    UINT32 update_id;
    UINT32 surface_id;
} QXLUpdateCmd;

typedef struct ATTR_PACKED QXLCursor {
    CursorHeader header;
    UINT32 data_size;
    QXLDataChunk chunk;
} QXLCursor;

enum {
    QXL_CURSOR_SET,
    QXL_CURSOR_MOVE,
    QXL_CURSOR_HIDE,
    QXL_CURSOR_TRAIL,
};

#define QXL_CURSUR_DEVICE_DATA_SIZE 128

typedef struct ATTR_PACKED QXLCursorCmd {
    QXLReleaseInfo release_info;
    UINT8 type;
    union {
        struct ATTR_PACKED {
            Point16 position;
            UINT8 visible;
            PHYSICAL shape;
        } set;
        struct ATTR_PACKED {
            UINT16 length;
            UINT16 frequency;
        } trail;
        Point16 position;
    } u;
    UINT8 device_data[QXL_CURSUR_DEVICE_DATA_SIZE]; //todo: use host memory
} QXLCursorCmd;

enum {
    QXL_DRAW_NOP,
    QXL_DRAW_FILL,
    QXL_DRAW_OPAQUE,
    QXL_DRAW_COPY,
    QXL_COPY_BITS,
    QXL_DRAW_BLEND,
    QXL_DRAW_BLACKNESS,
    QXL_DRAW_WHITENESS,
    QXL_DRAW_INVERS,
    QXL_DRAW_ROP3,
    QXL_DRAW_STROKE,
    QXL_DRAW_TEXT,
    QXL_DRAW_TRANSPARENT,
    QXL_DRAW_ALPHA_BLEND,
};

typedef struct ATTR_PACKED QXLString{
    UINT32 data_size;
    UINT16 length;
    UINT16 flags;
    QXLDataChunk chunk;
} QXLString;

typedef struct ATTR_PACKED QXLCopyBits {
    Point src_pos;
} QXLCopyBits;

#define QXL_EFFECT_BLEND 0
#define QXL_EFFECT_OPAQUE 1
#define QXL_EFFECT_REVERT_ON_DUP 2
#define QXL_EFFECT_BLACKNESS_ON_DUP 3
#define QXL_EFFECT_WHITENESS_ON_DUP 4
#define QXL_EFFECT_NOP_ON_DUP 5
#define QXL_EFFECT_NOP 6
#define QXL_EFFECT_OPAQUE_BRUSH 7

typedef struct ATTR_PACKED QXLDrawable {
    QXLReleaseInfo release_info;
    uint32_t surface_id;
    UINT8 effect;
    UINT8 type;
    UINT8 self_bitmap;
    Rect self_bitmap_area;
    Rect bbox;
    Clip clip;
    UINT32 mm_time;
    INT32 surfaces_dest[3];
    Rect surfaces_rects[3];
    union {
        Fill fill;
        Opaque opaque;
        Copy copy;
        Transparent transparent;
        AlphaBlnd alpha_blend;
        QXLCopyBits copy_bits;
        Blend blend;
        Rop3 rop3;
        Stroke stroke;
        Text text;
        Blackness blackness;
        Invers invers;
        Whiteness whiteness;
    } u;
} QXLDrawable;

typedef struct ATTR_PACKED QXLClipRects{
    UINT32 num_rects;
    QXLDataChunk chunk;
} QXLClipRects;

enum {
    QXL_PATH_BEGIN = (1 << 0),
    QXL_PATH_END = (1 << 1),
    QXL_PATH_CLOSE = (1 << 3),
    QXL_PATH_BEZIER = (1 << 4), 
};

typedef struct ATTR_PACKED QXLPath {
    UINT32 data_size;
    QXLDataChunk chunk;
} QXLPath;

enum {
    QXL_IMAGE_GROUP_DRIVER,
    QXL_IMAGE_GROUP_DEVICE,
    QXL_IMAGE_GROUP_RED,
    QXL_IMAGE_GROUP_DRIVER_DONT_CACHE,
};

typedef struct ATTR_PACKED QXLImageID{
    UINT32 group;
    UINT32 unique;
} QXLImageID;

enum {
    QXL_IMAGE_CACHE = (1 << 0),
};

enum {
    QXL_BITMAP_DIRECT = (1 << 0),
    QXL_BITMAP_UNSTABLE = (1 << 1),
    QXL_BITMAP_TOP_DOWN = (1 << 2), // == BITMAP_TOP_DOWN
};

#define QXL_SET_IMAGE_ID(image, _group, _unique) {              \
    UINT64* id_ptr = &(image)->descriptor.id;                   \
    QXLImageID *image_id = (QXLImageID *)id_ptr;                \
    image_id->group = _group;                                   \
    image_id->unique = _unique;                                 \
}

typedef struct ATTR_PACKED QXLImage {
    ImageDescriptor descriptor;
    union { // variable length
        Bitmap bitmap;
        PNGData png;
        QUICData quic;
    };    
} QXLImage;

#define VDI_PORT_DEVICE_ID 0x0105
#define VDI_PORT_REVISION 0x01

#define VDI_PORT_INTERRUPT (1 << 0)

#define VDI_PORT_MAGIC (*(UINT32*)"VDIP")

typedef struct ATTR_PACKED VDIPortPacket {
    UINT32 gen;
    UINT32 size;
    UINT8 data[512 - 2 * sizeof(UINT32)];
} VDIPortPacket;

RING_DECLARE(VDIPortRing, VDIPortPacket, 32);

enum {
    VDI_PORT_IO_RANGE_INDEX,
    VDI_PORT_RAM_RANGE_INDEX,
};

enum {
    VDI_PORT_IO_CONNECTION,
    VDI_PORT_IO_NOTIFY = 4,
    VDI_PORT_IO_UPDATE_IRQ = 8,

    VDI_PORT_IO_RANGE_SIZE = 12
};

typedef struct ATTR_PACKED VDIPortRam {
    UINT32 magic;
    UINT32 generation;
    UINT32 int_pending;
    UINT32 int_mask;
    VDIPortRing input;
    VDIPortRing output;
    UINT32 reserv[32];
} VDIPortRam;


#ifndef __GNUC__
#pragma pack(pop)
#endif

#undef ATTR_PACKED

#endif
