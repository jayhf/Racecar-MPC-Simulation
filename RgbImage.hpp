#include <memory>

#pragma once

#include <string>
#include <memory>
#include <fstream>
#include <utility>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>

/// Stores a pixel in RGBA uint8 format
struct Pixel{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;
};

/// A helper class for working with RGBA images that can load, save and convert them to OpenGL textures
class RgbImage {
public:
    RgbImage(unsigned int width, unsigned int height, std::unique_ptr<Pixel[]> &&buffer);
    explicit RgbImage(const char* path);
    RgbImage(const RgbImage&) = delete;
    RgbImage(RgbImage&& img) = default;

    /// Creates an image of a certain size. The data is not initialized.
    /// \param width The width of the image in pixels
    /// \param height The height of the iamge in pixels
    RgbImage(unsigned int width, unsigned int height);

    /// Makes an image from any rectangle on the current OpenGL context from the specified position
    /// \param x The x coordinate of the top left corner
    /// \param y The y coordinate of the top left corner
    /// \param width The width of the rectangle
    /// \param height The height of the rectangle
    RgbImage(GLint x, GLint y, GLsizei width, GLsizei height);

    /// Saves the images as a P6 .ppm file
    /// \param path The path to save the image in
    void save_ppm(const char *path);

    //Not worth debugging build system issues. Easier to just convert ppm to jpeg with Photoshop.
    //void save_jpeg(const char *path);

    /// Creates an OpenGL texture from this image
    /// \param wrap The wrap setting for both vertical and horizontal directions. Can be any of GL_TEXTURE_WRAP_
    /// \return the OpenGL texture
    GLuint create_texture(GLint wrap);

    /// \return The image width in pixels
    unsigned int get_width() const;

    /// \return The image height in pixels
    unsigned int get_height() const;

    /// \return a pointer to the internal pixel buffer
    Pixel* get_buffer() const;

    /// \return The total number of pixels in the image
    unsigned size() const;

private:
    unsigned int _width{};
    unsigned int _height{};
    std::unique_ptr<Pixel[]> _buffer;
};