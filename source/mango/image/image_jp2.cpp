/*
    MANGO Multimedia Development Platform
    Copyright (C) 2012-2024 Twilight Finland 3D Oy Ltd. All rights reserved.
*/
#include <mango/core/core.hpp>
#include <mango/image/image.hpp>

#if defined(MANGO_ENABLE_JP2)

#include <openjpeg.h>

namespace
{
    using namespace mango;
    using namespace mango::image;

    // ------------------------------------------------------------
    // OpenJPEG interface
    // ------------------------------------------------------------

    struct MemoryStreamReader
    {
        ConstMemory memory;
        size_t offset;
        opj_stream_t* stream;

        MemoryStreamReader(ConstMemory memory)
            : memory(memory)
            , offset(0)
            , stream(nullptr)
        {
            constexpr bool is_reader = true;

            stream = opj_stream_default_create(is_reader);
            if (stream)
            {
                opj_stream_set_read_function(stream, stream_read);
                opj_stream_set_seek_function(stream, stream_seek);
                opj_stream_set_skip_function(stream, stream_skip);
                opj_stream_set_user_data(stream, this, stream_free_user_data);
                opj_stream_set_user_data_length(stream, memory.size);
            }
        }

        ~MemoryStreamReader()
        {
            if (stream)
            {
                opj_stream_destroy(stream);
            }
        }

        static
        OPJ_SIZE_T stream_read(void* buffer, OPJ_SIZE_T bytes, void* data)
        {
            MemoryStreamReader& reader = *reinterpret_cast<MemoryStreamReader*>(data);
            ConstMemory& memory = reader.memory;

            if (reader.offset >= memory.size)
                return (OPJ_SIZE_T) -1;

            if (bytes > (memory.size - reader.offset))
                bytes = memory.size - reader.offset;

            std::memcpy(buffer, memory + reader.offset, bytes);
            reader.offset += bytes;

            return bytes;
        }

        static
        OPJ_OFF_T stream_skip(OPJ_OFF_T bytes, void* data)
        {
            MemoryStreamReader& reader = *reinterpret_cast<MemoryStreamReader*>(data);
            ConstMemory& memory = reader.memory;

            if (bytes < 0)
                return -1;

            OPJ_OFF_T left = memory.size - reader.offset;
            if (bytes > left)
                bytes = left;

            reader.offset += bytes;

            return bytes;
        }

        static
        OPJ_BOOL stream_seek(OPJ_OFF_T bytes, void* data)
        {
            MemoryStreamReader& reader = *reinterpret_cast<MemoryStreamReader*>(data);
            ConstMemory& memory = reader.memory;

            if (bytes < 0 || bytes > (OPJ_OFF_T)memory.size)
                return OPJ_FALSE;

            reader.offset = (OPJ_SIZE_T)bytes;

            return OPJ_TRUE;
        }

        static
        void stream_free_user_data(void* data)
        {
            MANGO_UNREFERENCED(data);
        }
    };

    struct StreamWriter
    {
        Stream& output;
        opj_stream_t* stream;

        StreamWriter(Stream& output)
            : output(output)
            , stream(nullptr)
        {
            constexpr bool is_reader = false;

            stream = opj_stream_default_create(is_reader);
            if (stream)
            {
                opj_stream_set_write_function(stream, stream_write);
                opj_stream_set_seek_function(stream, stream_seek);
                opj_stream_set_skip_function(stream, stream_skip);
                opj_stream_set_user_data(stream, &output, stream_free_user_data);
            }
        }

        ~StreamWriter()
        {
            if (stream)
            {
                opj_stream_destroy(stream);
            }
        }

        static
        OPJ_SIZE_T stream_write(void* buffer, OPJ_SIZE_T bytes, void* data)
        {
            Stream& output = *reinterpret_cast<Stream*>(data);
            output.write(buffer, bytes);
            return bytes;
        }

        static
        OPJ_OFF_T stream_skip(OPJ_OFF_T bytes, void* data)
        {
            Stream& output = *reinterpret_cast<Stream*>(data);
            output.seek(bytes, Stream::CURRENT);
            return bytes;
        }

        static
        OPJ_BOOL stream_seek(OPJ_OFF_T bytes, void* data)
        {
            Stream& output = *reinterpret_cast<Stream*>(data);
            output.seek(bytes, Stream::BEGIN);
            return true;
        }

        static
        void stream_free_user_data(void* data)
        {
            MANGO_UNREFERENCED(data);
        }
    };

    struct CallbackManager
    {
        int level; // 0: none, 1: error, 2: error + warning, 3: everything

        CallbackManager(opj_codec_t* codec, int level)
            : level(level)
        {
            opj_set_info_handler(codec, info_callback, this);
            opj_set_warning_handler(codec, warning_callback, this);
            opj_set_error_handler(codec, error_callback, this);
        }

        static
        void error_callback(const char *msg, void* data)
        {
            CallbackManager& manager = *reinterpret_cast<CallbackManager*>(data);
            if (manager.level >= 1)
            {
                fprintf(stdout, "[ERROR] %s", msg);
            }
        }

        static
        void warning_callback(const char *msg, void *data)
        {
            CallbackManager& manager = *reinterpret_cast<CallbackManager*>(data);
            if (manager.level >= 2)
            {
                fprintf(stdout, "[WARNING] %s", msg);
            }
        }

        static
        void info_callback(const char *msg, void *data)
        {
            CallbackManager& manager = *reinterpret_cast<CallbackManager*>(data);
            if (manager.level >= 3)
            {
                fprintf(stdout, "[INFO] %s", msg);
            }
        }
    };

    static constexpr
    u8 JP2_RFC3745_MAGIC [] =
    {
        0x00, 0x00, 0x00, 0x0c,
        0x6a, 0x50, 0x20, 0x20,
        0x0d, 0x0a, 0x87, 0x0a
    };

    static constexpr
    u8 J2K_CODESTREAM_MAGIC [] =
    {
        0xff, 0x4f, 0xff, 0x51
    };

    static
    opj_codec_t* create_codec(ConstMemory memory)
    {
        if (memory.size < 12)
        {
            return nullptr;
        }

        printLine(Print::Info, "[header]");
        std::string ms = "  magic: | ";
        for (int i = 0; i < 12; ++i)
        {
            ms += fmt::format("{:#04x}", memory[i]);
            if (!((i - 3) % 4))
            {
                ms += "| ";

            }
        }
        printLine(Print::Info, ms);

        opj_codec_t* codec = nullptr;

        if (!std::memcmp(memory.address, JP2_RFC3745_MAGIC, 12) ||
            !std::memcmp(memory.address, JP2_RFC3745_MAGIC + 8, 4))
        {
            printLine(Print::Info, "  codec: JP2");
            codec = opj_create_decompress(OPJ_CODEC_JP2);
        }
        else if (!std::memcmp(memory.address, J2K_CODESTREAM_MAGIC, 4))
        {
            printLine(Print::Info, "  codec: J2K");
            codec = opj_create_decompress(OPJ_CODEC_J2K);
        }

        return codec;
    }

    static
    opj_image_t* to_opj_image(const Surface& surface, int nr_comp, int sub_dx, int sub_dy)
    {
        int width = surface.width;
        int height = surface.height;

        opj_image_cmptparm_t cmptparm[4];
        std::memset(cmptparm, 0, nr_comp * sizeof(opj_image_cmptparm_t));

        for (int comp = 0; comp < nr_comp; ++comp)
        {
            cmptparm[comp].prec = 8;
            cmptparm[comp].sgnd = 0;
            cmptparm[comp].dx = sub_dx;
            cmptparm[comp].dy = sub_dy;
            cmptparm[comp].w = width;
            cmptparm[comp].h = height;
        }

        OPJ_COLOR_SPACE color_space = nr_comp <= 2 ? OPJ_CLRSPC_GRAY : OPJ_CLRSPC_SRGB;

        opj_image_t* image = opj_image_create(nr_comp, &cmptparm[0], color_space);
        if (!image)
        {
            return nullptr;
        }

        image->x0 = 0;
        image->y0 = 0;
        image->x1 = (width - 1) * sub_dx + 1;
        image->y1 = (height - 1) * sub_dy + 1;

        const u8* buffer = surface.image;
        size_t stride = surface.stride;

        size_t offset = 0;

        for (int y = 0; y < height; ++y)
        {
            const u8* src = buffer;

            for (int x = 0; x < width; ++x)
            {
                for (int ch = 0; ch < nr_comp; ++ch)
                {
                    image->comps[ch].data[offset] = src[ch];
                }

                ++offset;
                src += nr_comp;
            }

            buffer += stride;
        }

        return image;
    }

    using ImageProcessFunc = void (*)(const Surface& surface, const opj_image_t& image);

    static
    void process_generic_1_comp(const Surface& surface, const opj_image_t& image)
    {
        int width = surface.width;
        int height = surface.height;
        size_t stride = surface.stride;

        for (int y = 0; y < height; ++y)
        {
            u8* dest = surface.image + y * stride;

            s32* src0 = image.comps[0].data + (y / image.comps[0].dy) * image.comps[0].w;
            u32 prec0 = image.comps[0].prec;
            u32 bias0 = image.comps[0].sgnd ? 1 << (prec0 - 1) : 0;

            for (int x = 0; x < width; ++x)
            {
                u32 s = src0[x / image.comps[0].dx];
                s = u32_scale(s + bias0, prec0, 8);
                dest[x] = u8(s);
            }
        }
    }

    static
    void process_generic_2_comp(const Surface& surface, const opj_image_t& image)
    {
        int width = surface.width;
        int height = surface.height;
        size_t stride = surface.stride;

        for (int y = 0; y < height; ++y)
        {
            u16* dest = reinterpret_cast<u16*>(surface.image + y * stride);

            s32* src0 = image.comps[0].data + (y / image.comps[0].dy) * image.comps[0].w;
            s32* src1 = image.comps[1].data + (y / image.comps[1].dy) * image.comps[1].w;

            u32 prec0 = image.comps[0].prec;
            u32 prec1 = image.comps[1].prec;

            u32 bias0 = image.comps[0].sgnd ? 1 << (prec0 - 1) : 0;
            u32 bias1 = image.comps[1].sgnd ? 1 << (prec1 - 1) : 0;

            for (int x = 0; x < width; ++x)
            {
                u32 s = src0[x / image.comps[0].dx];
                u32 a = src1[x / image.comps[1].dx];
                s = u32_scale(s + bias0, prec0, 8);
                a = u32_scale(a + bias1, prec1, 8);
                dest[x] = u16((a << 8) | s);
            }
        }
    }

    static
    void process_generic_3_comp(const Surface& surface, const opj_image_t& image)
    {
        int width = surface.width;
        int height = surface.height;
        size_t stride = surface.stride;

        bool is_yuv = (image.comps[0].dx == 1 && image.comps[0].dy == 1) &&
                       image.comps[1].dx != 1;

        for (int y = 0; y < height; ++y)
        {
            u32* dest = reinterpret_cast<u32*>(surface.image + y * stride);

            s32* src0 = image.comps[0].data + (y / image.comps[0].dy) * image.comps[0].w;
            s32* src1 = image.comps[1].data + (y / image.comps[1].dy) * image.comps[1].w;
            s32* src2 = image.comps[2].data + (y / image.comps[2].dy) * image.comps[2].w;

            u32 prec0 = image.comps[0].prec;
            u32 prec1 = image.comps[1].prec;
            u32 prec2 = image.comps[2].prec;

            u32 bias0 = image.comps[0].sgnd ? 1 << (prec0 - 1) : 0;
            u32 bias1 = image.comps[1].sgnd ? 1 << (prec1 - 1) : 0;
            u32 bias2 = image.comps[2].sgnd ? 1 << (prec2 - 1) : 0;

            for (int x = 0; x < width; ++x)
            {
                u32 s0 = src0[x / image.comps[0].dx];
                u32 s1 = src1[x / image.comps[1].dx];
                u32 s2 = src2[x / image.comps[2].dx];

                s0 = u32_scale(s0 + bias0, prec0, 8);
                s1 = u32_scale(s1 + bias1, prec1, 8);
                s2 = u32_scale(s2 + bias2, prec2, 8);

                u32 r = s0;
                u32 g = s1;
                u32 b = s2;
                u32 a = 0xff;

                if (is_yuv)
                {
                    s32 cb = s1;
                    s32 cr = s2;
                    r = s0 + ((cr * 91750 - 11711232) >> 16);
                    g = s0 + ((cb * -22479 + cr * -46596 + 8874368) >> 16);
                    b = s0 + ((cb * 115671 - 14773120) >> 16);
                    r = byteclamp(r);
                    g = byteclamp(g);
                    b = byteclamp(b);
                }

                dest[x] = makeRGBA(r, g, b, a);
            }
        }
    }

    static
    void process_generic_4_comp(const Surface& surface, const opj_image_t& image)
    {
        int width = surface.width;
        int height = surface.height;
        size_t stride = surface.stride;

        bool is_yuv = (image.comps[0].dx == 1 && image.comps[0].dy == 1) &&
                       image.comps[1].dx != 1;

        for (int y = 0; y < height; ++y)
        {
            u32* dest = reinterpret_cast<u32*>(surface.image + y * stride);

            s32* src0 = image.comps[0].data + (y / image.comps[0].dy) * image.comps[0].w;
            s32* src1 = image.comps[1].data + (y / image.comps[1].dy) * image.comps[1].w;
            s32* src2 = image.comps[2].data + (y / image.comps[2].dy) * image.comps[2].w;
            s32* src3 = image.comps[3].data + (y / image.comps[3].dy) * image.comps[3].w;

            u32 prec0 = image.comps[0].prec;
            u32 prec1 = image.comps[1].prec;
            u32 prec2 = image.comps[2].prec;
            u32 prec3 = image.comps[3].prec;

            u32 bias0 = image.comps[0].sgnd ? 1 << (prec0 - 1) : 0;
            u32 bias1 = image.comps[1].sgnd ? 1 << (prec1 - 1) : 0;
            u32 bias2 = image.comps[2].sgnd ? 1 << (prec2 - 1) : 0;
            u32 bias3 = image.comps[3].sgnd ? 1 << (prec3 - 1) : 0;

            for (int x = 0; x < width; ++x)
            {
                u32 s0 = src0[x / image.comps[0].dx];
                u32 s1 = src1[x / image.comps[1].dx];
                u32 s2 = src2[x / image.comps[2].dx];
                u32 s3 = src3[x / image.comps[3].dx];

                s0 = u32_scale(s0 + bias0, prec0, 8);
                s1 = u32_scale(s1 + bias1, prec1, 8);
                s2 = u32_scale(s2 + bias2, prec2, 8);
                s3 = u32_scale(s3 + bias3, prec3, 8);

                u32 r = s0;
                u32 g = s1;
                u32 b = s2;
                u32 a = s3;

                if (is_yuv)
                {
                    s32 cb = s1;
                    s32 cr = s2;
                    r = s0 + ((cr * 91750 - 11711232) >> 16);
                    g = s0 + ((cb * -22479 + cr * -46596 + 8874368) >> 16);
                    b = s0 + ((cb * 115671 - 14773120) >> 16);
                    r = byteclamp(r);
                    g = byteclamp(g);
                    b = byteclamp(b);
                }

                dest[x] = makeRGBA(r, g, b, a);
            }
        }
    }

    static
    void process_unorm_8bit_y(const Surface& surface, const opj_image_t& image)
    {
        int width = surface.width;
        int height = surface.height;
        size_t stride = surface.stride;

        for (int y = 0; y < height; ++y)
        {
            u8* dest = surface.image + y * stride;
            s32* src0 = image.comps[0].data + y * image.comps[0].w;

            for (int x = 0; x < width; ++x)
            {
                u32 s = src0[x];
                dest[x] = u8(s);
            }
        }
    }

    static
    void process_unorm_8bit_ya(const Surface& surface, const opj_image_t& image)
    {
        int width = surface.width;
        int height = surface.height;
        size_t stride = surface.stride;

        for (int y = 0; y < height; ++y)
        {
            u16* dest = reinterpret_cast<u16*>(surface.image + y * stride);
            s32* src0 = image.comps[0].data + y * image.comps[0].w;
            s32* src1 = image.comps[1].data + y * image.comps[1].w;

            for (int x = 0; x < width; ++x)
            {
                u32 s = src0[x];
                u32 a = src1[x];
                dest[x] = u16((a << 8) | s);
            }
        }
    }

    static
    void process_unorm_8bit_rgb(const Surface& surface, const opj_image_t& image)
    {
        int width = surface.width;
        int height = surface.height;
        size_t stride = surface.stride;

        for (int y = 0; y < height; ++y)
        {
            u32* dest = reinterpret_cast<u32*>(surface.image + y * stride);

            s32* src0 = image.comps[0].data + y * image.comps[0].w;
            s32* src1 = image.comps[1].data + y * image.comps[1].w;
            s32* src2 = image.comps[2].data + y * image.comps[2].w;

            for (int x = 0; x < width; ++x)
            {
                s32 r = src0[x];
                s32 g = src1[x];
                s32 b = src2[x];
                s32 a = 0xff;
                dest[x] = makeRGBA(r, g, b, a);
            }
        }
    }

    static
    void process_unorm_8bit_rgba(const Surface& surface, const opj_image_t& image)
    {
        int width = surface.width;
        int height = surface.height;
        size_t stride = surface.stride;

        for (int y = 0; y < height; ++y)
        {
            u32* dest = reinterpret_cast<u32*>(surface.image + y * stride);

            s32* src0 = image.comps[0].data + y * image.comps[0].w;
            s32* src1 = image.comps[1].data + y * image.comps[1].w;
            s32* src2 = image.comps[2].data + y * image.comps[2].w;
            s32* src3 = image.comps[3].data + y * image.comps[3].w;

            for (int x = 0; x < width; ++x)
            {
                s32 r = src0[x];
                s32 g = src1[x];
                s32 b = src2[x];
                s32 a = src3[x];
                dest[x] = makeRGBA(r, g, b, a);
            }
        }
    }

#if 0

    static
    void process_unorm_8bit_yuv(const Surface& surface, const opj_image_t& image)
    {
        int width = surface.width;
        int height = surface.height;
        size_t stride = surface.stride;

        for (int y = 0; y < height; ++y)
        {
            u32* dest = reinterpret_cast<u32*>(surface.image + y * stride);

            s32* src0 = image.comps[0].data + y * image.comps[0].w;
            s32* src1 = image.comps[1].data + y * image.comps[1].w;
            s32* src2 = image.comps[2].data + y * image.comps[2].w;

            for (int x = 0; x < width; ++x)
            {
                s32 s0 = src0[x];
                s32 cb = src1[x];
                s32 cr = src2[x];

                s32 r = s0 + ((cr * 91750 - 11711232) >> 16);
                s32 g = s0 + ((cb * -22479 + cr * -46596 + 8874368) >> 16);
                s32 b = s0 + ((cb * 115671 - 14773120) >> 16);
                r = byteclamp(r);
                g = byteclamp(g);
                b = byteclamp(b);

                s32 a = 0xff;

                dest[x] = makeRGBA(r, g, b, a);
            }
        }
    }

#endif

    // ------------------------------------------------------------
    // ImageDecoder
    // ------------------------------------------------------------

    struct Interface : ImageDecodeInterface
    {
        ImageProcessFunc m_process_func = nullptr;
        MemoryStreamReader m_memory_reader;

        opj_codec_t* m_codec = nullptr;
        opj_stream_t* m_stream = nullptr;
        opj_image_t* m_image = nullptr;

        Interface(ConstMemory memory)
            : m_memory_reader(memory)
        {
            m_codec = create_codec(memory);
            if (!m_codec)
            {
                header.setError("[ImageDecoder.JP2] Incorrect identifier.");
                return;
            }

            m_stream = m_memory_reader.stream;
            if (!m_stream)
            {
                header.setError("[ImageDecoder.JP2] opj_stream_create_default_memory_stream FAILED.");
                return;
            }

            opj_dparameters_t parameters;
            opj_set_default_decoder_parameters(&parameters);

            if (!opj_setup_decoder(m_codec, &parameters))
            {
                header.setError("[ImageDecoder.JP2] opj_setup_decoder FAILED.");
                return;
            }

            CallbackManager callback(m_codec, 0);

            size_t num_thread = std::max(std::thread::hardware_concurrency(), 1u);
            opj_codec_set_threads(m_codec, int(num_thread));

            if (!opj_read_header(m_stream, m_codec, &m_image))
            {
                header.setError("[ImageDecoder.JP2] opj_read_header FAILED.");
                return;
            }

            if (!m_image)
            {
                header.setError("[ImageDecoder.JP2] Incorrect image.");
                return;
            }

            // ICC color profile
            icc.address = m_image->icc_profile_buf;
            icc.size = m_image->icc_profile_len;

            u32 width = m_image->x1 - m_image->x0;
            u32 height = m_image->y1 - m_image->y0;
            u32 components = m_image->numcomps;

            Format format;

            switch (components)
            {
                case 1:
                    format = LuminanceFormat(8, Format::UNORM, 8, 0);
                    m_process_func = process_generic_1_comp;
                    break;

                case 2:
                    format = LuminanceFormat(16, Format::UNORM, 8, 8);
                    m_process_func = process_generic_2_comp;
                    break;

                case 3:
                    format = Format(32, Format::UNORM, Format::RGBA, 8, 8, 8, 8);
                    m_process_func = process_generic_3_comp;
                    break;

                case 4:
                    format = Format(32, Format::UNORM, Format::RGBA, 8, 8, 8, 8);
                    m_process_func = process_generic_4_comp;
                    break;

                default:
                    header.setError("[ImageDecoder.JP2] Incorrect number of components ({}).", components);
                    return;
            }

            printLine(Print::Info, "[image]");
            printLine(Print::Info, "  dimensions: {} x {}", width, height);
            printLine(Print::Info, "  color space: {}", int(m_image->color_space));

            printLine(Print::Info, "[components]");

            bool is_signed = false;
            bool is_subsampled = false;
            bool is_8bit = true;

            for (u32 i = 0; i < components; ++i)
            {
                const opj_image_comp_t& comp = m_image->comps[i];

                printLine(Print::Info, "  #{}: {} x {}, bits: {}, alpha: {}, sgnd: {}, dx: {}, dy: {}", 
                    i, comp.w, comp.h, comp.prec, comp.alpha, comp.sgnd, comp.dx, comp.dy);

                if (comp.w != width || comp.h != height || comp.dx != 1 || comp.dy != 1)
                {
                    is_subsampled = true;
                }

                if (comp.sgnd)
                {
                    is_signed = true;
                }

                if (comp.prec != 8)
                {
                    is_8bit = false;
                }
            }

            bool is_standard = is_8bit && !is_signed && !is_subsampled;

            switch (m_image->color_space)
            {
                case OPJ_CLRSPC_UNKNOWN:
                    header.setError("[ImageDecoder.JP2] Unknown color space ({}).", int(m_image->color_space));
                    return;

                case OPJ_CLRSPC_UNSPECIFIED:
                    // Determine heuristically
                    switch (components)
                    {
                        case 1:
                            if (is_standard)
                            {
                                m_process_func = process_unorm_8bit_y;
                            }
                            break;
                        case 2:
                            if (is_standard)
                            {
                                m_process_func = process_unorm_8bit_ya;
                            }
                            break;
                        case 3:
                            if (is_standard)
                            {
                                m_process_func = process_unorm_8bit_rgb;
                            }
                            break;
                        case 4:
                            if (is_standard)
                            {
                                m_process_func = process_unorm_8bit_rgba;
                            }
                            break;
                    }
                    break;

                case OPJ_CLRSPC_SRGB:
                    if (components < 3)
                    {
                        header.setError("[ImageDecoder.JP2] Incorrect number of components ({}).", components);
                        return;
                    }
                    if (is_standard)
                    {
                        if (components == 3)
                            m_process_func = process_unorm_8bit_rgb;
                        else
                            m_process_func = process_unorm_8bit_rgba;
                    }
                    break;

                case OPJ_CLRSPC_GRAY:
                    if (components > 2)
                    {
                        header.setError("[ImageDecoder.JP2] Incorrect number of components ({}).", components);
                        return;
                    }
                    if (is_standard)
                    {
                        if (components == 1)
                        {
                            m_process_func = process_unorm_8bit_y;
                        }
                        else
                        {
                            m_process_func = process_unorm_8bit_ya;
                        }
                    }
                    break;

                case OPJ_CLRSPC_SYCC:
                case OPJ_CLRSPC_EYCC:
                case OPJ_CLRSPC_CMYK:
                    header.setError("[ImageDecoder.JP2] Unsupported color space ({}).", int(m_image->color_space));
                    return;

                default:
                    header.setError("[ImageDecoder.JP2] Incorrect color space ({}).", int(m_image->color_space));
                    return;
            }

            header.width   = width;
            header.height  = height;
            header.depth   = 0;
            header.levels  = 0;
            header.faces   = 0;
            header.palette = false;
            header.format  = format;
            header.compression = TextureCompression::NONE;
        }

        ~Interface()
        {
            if (m_image)
            {
                opj_image_destroy(m_image);
            }

            if (m_codec)
            {
                opj_destroy_codec(m_codec);
            }
        }

        ImageDecodeStatus decode(const Surface& dest, const ImageDecodeOptions& options, int level, int depth, int face) override
        {
            MANGO_UNREFERENCED(level);
            MANGO_UNREFERENCED(depth);
            MANGO_UNREFERENCED(face);

            if (options.multithread)
            {
                // NOTE: It is too late to configure the number of threads here;
                //       it has to be done before opj_read_header is called.
            }

            ImageDecodeStatus status;

            if (!opj_set_decode_area(m_codec, m_image, 0, 0, 0, 0))
            {
                status.setError("[ImageDecoder.JP2] opj_set_decode_area FAILED.");
                return status;
            }

            if (!opj_decode(m_codec, m_stream, m_image))
            {
                status.setError("[ImageDecoder.JP2] opj_decode FAILED.");
                return status;
            }

            if (!opj_end_decompress(m_codec, m_stream))
            {
                status.setError("[ImageDecoder.JP2] opj_end_decompress FAILED.");
                return status;
            }

            if (m_process_func)
            {
                Bitmap bitmap(header.width, header.height, header.format);
                m_process_func(bitmap, *m_image);
                dest.blit(0, 0, bitmap);
            }

            return status;
        }
    };

    ImageDecodeInterface* createInterface(ConstMemory memory)
    {
        ImageDecodeInterface* x = new Interface(memory);
        return x;
    }

    // ------------------------------------------------------------
    // ImageEncoder
    // ------------------------------------------------------------

    ImageEncodeStatus imageEncode(Stream& output, const Surface& surface, const ImageEncodeOptions& options)
    {
        Format format(32, Format::UNORM, Format::RGBA, 8, 8, 8, 8);
        Bitmap bitmap(surface, format);

        int numcomps = 4;

        opj_cparameters_t parameters;
        opj_set_default_encoder_parameters(&parameters);

        char comment[] = "Created by MANGO OpenJPEG encoder.";

        if (!parameters.cp_comment)
        {
            parameters.cp_comment = comment;
        }

        if (!parameters.tcp_numlayers)
        {
            parameters.tcp_rates[0] = 0;
            parameters.tcp_numlayers++;
            parameters.cp_disto_alloc = 1;
        }

        if (options.lossless)
        {
            parameters.irreversible = 0;
        }
        else
        {
            parameters.irreversible = 1;
            parameters.tcp_numlayers = 1;
            parameters.cp_fixed_quality = 1;
            parameters.tcp_distoratio[0] = u32(std::clamp(options.quality, 0.0f, 1.0f) * 100.0f);
        }

        ImageEncodeStatus status;

        int sub_dx = parameters.subsampling_dx;
        int sub_dy = parameters.subsampling_dy;

        opj_image_t* image = to_opj_image(bitmap, numcomps, sub_dx, sub_dy);
        if (!image)
        {
            status.setError("[ImageEncoder.JP2] to_opj_image FAILED.");
            return status;
        }

        // ICC color profile
        if (options.icc.size > 0)
        {
            image->icc_profile_len = static_cast<OPJ_UINT32>(options.icc.size);
            image->icc_profile_buf = const_cast<u8*>(options.icc.address);
        }

        parameters.tcp_mct = image->numcomps == 3 ? 1 : 0;

        u32 s0 = std::clamp(u32_log2(bitmap.width) / 2 - 1, 1, 8);
        u32 s1 = std::clamp(u32_log2(bitmap.height) / 2 - 1, 1, 8);
        parameters.numresolution = std::max(s0, s1);

        OPJ_CODEC_FORMAT codec_format = OPJ_CODEC_JP2;

        opj_codec_t* codec = opj_create_compress(codec_format);
        if (!codec)
        {
            opj_image_destroy(image);
            status.setError("[ImageEncoder.JP2] opj_create_compress FAILED.");
            return status;
        }

        CallbackManager callback(codec, 0);

        if (options.multithread)
        {
            size_t num_thread = std::max(std::thread::hardware_concurrency(), 1u);
            opj_codec_set_threads(codec, int(num_thread));
        }

        opj_setup_encoder(codec, &parameters, image);

        StreamWriter writer(output);
        opj_stream_t* stream = writer.stream;
        if (!stream)
        {
            opj_image_destroy(image);
            opj_destroy_codec(codec);
            status.setError("[ImageEncoder.JP2] opj_stream_default_create FAILED.");
            return status;
        }

        if (!opj_start_compress(codec, image, stream))
        {
            opj_image_destroy(image);
            opj_destroy_codec(codec);
            status.setError("[ImageEncoder.JP2] opj_start_compress FAILED.");
            return status;
        }

        if (!opj_encode(codec, stream))
        {
            opj_image_destroy(image);
            opj_destroy_codec(codec);
            status.setError("[ImageEncoder.JP2] opj_encode FAILED.");
            return status;
        }

        opj_end_compress(codec, stream);

        opj_image_destroy(image);
        opj_destroy_codec(codec);

        printLine(Print::Info, "Encoded: {} bytes", writer.output.size());

        return status;
    }

} // namespace

namespace mango::image
{

    void registerImageCodecJP2()
    {
        registerImageDecoder(createInterface, ".jp2");
        registerImageDecoder(createInterface, ".j2k");
        registerImageDecoder(createInterface, ".j2c");
        registerImageDecoder(createInterface, ".jpc");
        registerImageDecoder(createInterface, ".jph");
        registerImageDecoder(createInterface, ".jhc");

        registerImageEncoder(imageEncode, ".jp2");
    }

} // namespace mango::image

#else

namespace mango::image
{

    void registerImageCodecJP2()
    {
    }

} // namespace mango::image

#endif // defined(MANGO_ENABLE_JP2)
