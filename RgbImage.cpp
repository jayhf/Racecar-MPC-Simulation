#include "RgbImage.hpp"

//#include <turbojpeg.h>
#include <iostream>

RgbImage::RgbImage(unsigned int width, unsigned int height, std::unique_ptr<Pixel[]> &&buffer)
        : _width(width),
          _height(height),
          _buffer(std::move(buffer)){}

RgbImage::RgbImage(const char *path) {
    std::ifstream in(path, std::ios::binary);

    char p;
    int version;
    int max_value;
    char nl;
    in >> p >> version >> _width >> _height >> max_value;
    if(p != 'P' || version != 6 || max_value != 255)
        throw std::domain_error("Unsupported format. Only supports PPM P6 files");

    _buffer = std::make_unique<Pixel[]>(size());
    Pixel pixel{};
    pixel.a = 255;
    for (int i = 0; i < size(); ++i) {
        in.read(reinterpret_cast<char*>(&pixel), 3);
        _buffer[i] = pixel;
    }
}

void RgbImage::save_ppm(const char *path) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << "P6\n" << _width << "\n" << _height << "\n" << 255 << "\n";
    for (int j = _height - 1; j >= 0; --j) {
        for (int i = 0; i < _width; ++i) {
            out.write(reinterpret_cast<char*>(&_buffer[i + j*_width]), 3);
        }
    }
}

GLuint RgbImage::create_texture(GLint wrap) {
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, _width, _height, 0, GL_RGBA, GL_UNSIGNED_BYTE, get_buffer());
    return texture;
}

RgbImage::RgbImage(unsigned int width, unsigned int height)
    : RgbImage(width, height, std::make_unique<Pixel[]>(width * height)){
}

RgbImage::RgbImage(GLint x, GLint y, GLsizei width, GLsizei height)
    : RgbImage(width, height){
    glReadPixels(x, y, width, height, GL_RGBA, GL_UNSIGNED_BYTE, get_buffer());
}

unsigned int RgbImage::get_width() const {
    return _width;
}

unsigned int RgbImage::get_height() const {
    return _height;
}

Pixel *RgbImage::get_buffer() const {
    return _buffer.get();
}

unsigned RgbImage::size() const {
    return _width*_height;
}

//void RgbImage::save_jpeg(const char *path) {
//    tjhandle tj = tjInitCompress();
//    if(tj == nullptr)
//        throw std::runtime_error("Error initializing turbo jpeg");
//
//    unsigned char* jpeg = nullptr;
//    unsigned long jpeg_size = 0;
//    if(0 > tjCompress2(tj, reinterpret_cast<unsigned char*>(&_buffer[0]), _width, 0, _height, TJPF_RGBA, &jpeg, &jpeg_size, TJSAMP_444, 50, TJFLAG_FASTDCT))
//        throw std::runtime_error("Error compressing image");
//    tjDestroy(tj);
//
//    //tj = tjInitTransform();
//    //tjTransform(tj, jpeg, jpeg_size, 1, )
//    std::ofstream out(path, std::ios::binary);
//    out.write(reinterpret_cast<char *>(jpeg), jpeg_size);
//    tjFree(jpeg);
//}
