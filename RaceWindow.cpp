#include "RaceWindow.hpp"

const char* CONE_MODEL_PATH = R"(C:\Users\Jay\PER_Copy_Test\blue_cone.STL)";
const char* CAR_MODEL_PATH = R"(C:\Users\Jay\Downloads\2018-03-31 REV4 TOP ASSY (1)\Exports\REV4.STL)";
const char* CONE_STRIPE_IMAGE_PATH = R"(C:\Users\Jay\PER_Copy_Test\Small White on Black.ppm)";

const char* IMAGE_OUTPUT_PATH = R"(C:\Users\Jay\PER_Copy_Test\Data\images\%06d.ppm)";
const char* LABEL2D_OUTPUT_PATH = R"(C:\Users\Jay\PER_Copy_Test\Data\labels\%06d.txt)";
const char* LABEL3D_OUTPUT_PATH = R"(C:\Users\Jay\PER_Copy_Test\Data\labels3d\%06d.txt)";

RaceWindow::RaceWindow()
{
    //Example hardcoded courses
    //int cone_count = 16;
    //for (int i = 0; i < cone_count; ++i) {
    //    float angle = i * 2*PI/cone_count;
    //    float c = std::cos(angle);
    //    float s = std::sin(angle);
    //    cones.emplace_back(11 * c, 11*s,
    //                       2*PI*rand()/static_cast<float>(RAND_MAX),
    //                       2*PI*rand()/static_cast<float>(RAND_MAX),
    //                       i == 0 ? Cone::Type::Orange : Cone::Type::Blue);
    //    cones.emplace_back(7.5 * c, 7.5*s,
    //                       2*PI*rand()/static_cast<float>(RAND_MAX),
    //                       2*PI*rand()/static_cast<float>(RAND_MAX),
    //                       i == 0 ? Cone::Type::Orange : Cone::Type::Yellow);
    //}
//
    //int accel_cone_count = 20;
    //for (int i = 0; i < accel_cone_count; ++i) {
    //    float x = 20 + i * 150/(accel_cone_count - 1);
    //    cones.emplace_back(x, 1.5f,
    //                       2*PI*rand()/static_cast<float>(RAND_MAX),
    //                       2*PI*rand()/static_cast<float>(RAND_MAX),
    //                       x >= 75.f || i == 0 ? Cone::Type::Orange : Cone::Type::Blue);
    //    cones.emplace_back(x, -1.5f,
    //                       2*PI*rand()/static_cast<float>(RAND_MAX),
    //                       2*PI*rand()/static_cast<float>(RAND_MAX),
    //                       x >= 75.f || i == 0 ? Cone::Type::Orange : Cone::Type::Yellow);
    //    if(i == accel_cone_count - 1){
    //        cones.emplace_back(x, .5f,
    //                           2*PI*rand()/static_cast<float>(RAND_MAX),
    //                           2*PI*rand()/static_cast<float>(RAND_MAX),
    //                           x >= 75.f || i == 0 ? Cone::Type::Orange : Cone::Type::Blue);
    //        cones.emplace_back(x, -.5f,
    //                           2*PI*rand()/static_cast<float>(RAND_MAX),
    //                           2*PI*rand()/static_cast<float>(RAND_MAX),
    //                           x >= 75.f || i == 0 ? Cone::Type::Orange : Cone::Type::Yellow);
    //    }
    //}

    //Build course from the specified trajectory
    auto trajectory = _simulation->get_trajectory();
    Float x = 0;
    while(x < 1.f){
        auto position = trajectory.path().position(x);
        auto offset = trajectory.path().derivative(x);
        offset.normalize();
        offset *= 1.5;

        Float next_x = trajectory.path().next_point(x, 3);
        cones.emplace_back(position[0] - offset[1], -(position[1] + offset[0]),
                           2*PI*rand()/static_cast<float>(RAND_MAX),
                           2*PI*rand()/static_cast<float>(RAND_MAX),
                           x == 0 || next_x >= 1 ? Cone::Type::Orange : Cone::Type::Blue);

        cones.emplace_back(position[0] + offset[1], -(position[1] - offset[0]),
                           2*PI*rand()/static_cast<float>(RAND_MAX),
                           2*PI*rand()/static_cast<float>(RAND_MAX),
                           x == 0 || next_x >= 1 ? Cone::Type::Orange : Cone::Type::Yellow);
        x = next_x;
    }


    set_default_size(400, 600);
    set_title("PER Racecar Simulation");

    add(windowBow);

    simulationArea.set_has_depth_buffer(true);
    simulationArea.set_hexpand(true);
    simulationArea.set_vexpand(true);
    simulationArea.set_auto_render(true);
    simulationArea.set_size_request(1920, 1080);
    simulationArea.show();
    windowBow.add(simulationArea);

    simulationArea.signal_realize().connect(sigc::mem_fun(*this, &RaceWindow::initOpenGl));
    simulationArea.signal_unrealize().connect(sigc::mem_fun(*this, &RaceWindow::deinitOpenGl), false);
    simulationArea.signal_render().connect(sigc::mem_fun(*this, &RaceWindow::render), false);
    simulationArea.set_can_focus(true);
    simulationArea.grab_focus();

    sliderBox.set_hexpand(true);
    for(auto axis : {std::make_tuple("X", 0), std::make_tuple("Y", 1), std::make_tuple("Z", 2)})
    {
        auto label = Gtk::manage(new Gtk::Label{std::get<0>(axis)});
        sliderBox.add(*label);
        label->show();

        auto range = Gtk::Adjustment::create(0.0, 0.0, 360.0, 1.0, 12.0, 0.0);

        range->signal_value_changed().connect(sigc::bind(
                sigc::mem_fun(*this, &RaceWindow::updateFromAdjustment), &viewAngles[std::get<1>(axis)], range));

        auto slider = Gtk::manage(new Gtk::Scale{range, Gtk::Orientation::ORIENTATION_HORIZONTAL});
        sliderBox.add(*slider);
        slider->set_hexpand(true);
        slider->show();
    }

    windowBow.add(sliderBox);
    sliderBox.property_margin() = 12;
    sliderBox.show();

    timeTextBuffer = Gtk::TextBuffer::create();
    timeText.set_buffer(timeTextBuffer);
    timeText.property_margin() = 12;
    timeText.show();
    windowBow.add(timeText);
    windowBow.show();

    tickCallback = gtk_widget_add_tick_callback(Widget::gobj(), staticTick, this, destroyNotify);

    this->signal_key_press_event().connect(
            sigc::mem_fun(*this, &RaceWindow::on_key_press), false);
    this->signal_key_release_event().connect(
            sigc::mem_fun(*this, &RaceWindow::on_key_press), false);
    this->signal_motion_notify_event().connect(
            sigc::mem_fun(*this, &RaceWindow::on_mouse_move), false);
    this->signal_scroll_event().connect(
            sigc::mem_fun(*this, &RaceWindow::on_mouse_scroll), false);

    this->add_events(
            Gdk::EXPOSURE_MASK
            | Gdk::POINTER_MOTION_MASK
            | Gdk::BUTTON_PRESS_MASK
            | Gdk::BUTTON_RELEASE_MASK
            | Gdk::BUTTON_MOTION_MASK
            | Gdk::POINTER_MOTION_HINT_MASK
            | Gdk::ENTER_NOTIFY_MASK
            | Gdk::LEAVE_NOTIFY_MASK);

    //maximize();
}

//TODO Need to figure out a better way of loading shaders
const char* cone_fs =
        "#version 330\n"
        "\n"
        "uniform vec4 coneColor;\n"
        "uniform vec3 lightDirection;\n"
        "uniform float stripeStart1;\n"
        "uniform float stripeStart2;\n"
        "uniform float stripeEnd1;\n"
        "uniform float stripeEnd2;\n"
        "uniform float stripeAngle;\n"
        "uniform sampler2D texture;\n"
        "\n"
        "in float height;\n"
        "in vec3 normal_fs;\n"
        "\n"
        "out vec4 outputColor;\n"
        "\n"
        "\n"
        "void main() {\n"
        "  float lerpVal = gl_FragCoord.y / 500.0f;\n"
        "\n"
        "  bool aboveStripe1 = height > stripeEnd1;\n"
        "  vec4 baseColor;\n"
        "  if(height < stripeStart1 || (height > stripeEnd1 && height < stripeStart2) || height > stripeEnd2){\n"
        "    baseColor = coneColor;\n"
        "  }\n"
        "  else{\n"
        "      float angle = stripeAngle - atan(normal_fs.z, normal_fs.x)*(1/(2*3.141593));\n"
        "      if(height < stripeEnd1){\n"
        "        baseColor = texture2D(texture, vec2(angle, (stripeEnd1 - height)/(stripeEnd1 - stripeStart1)));\n"
        "      }\n"
        "      else{\n"
        "        baseColor = texture2D(texture, vec2(angle, (stripeEnd2 - height)/(stripeEnd2 - stripeStart2)));\n"
        "      }\n"
        "  }\n"
        "\n"
        "  outputColor = vec4((baseColor * (.25 + max(0, .75 * dot(lightDirection, normal_fs)))).xyz, coneColor.w);"
        "}";

const char* cone_vs =
        "#version 330\n"
        "\n"
        "uniform mat4 mvp;\n"
        "\n"
        "layout(location = 0) in vec3 position;\n"
        "layout(location = 1) in vec3 normal;\n"
        "\n"
        "out float height;\n"
        "out vec3 normal_fs;\n"
        "\n"
        "void main() {\n"
        "  gl_Position = mvp * vec4(position.xyz, 1.0);\n"
        "  height = position.y;\n"
        "  normal_fs = normal;\n"
        "}";

const char* solid_color_fs =
        "#version 330\n"
        "\n"
        "uniform vec4 color;\n"
        "\n"
        "out vec4 outputColor;\n"
        "\n"
        "void main() {\n"
        "  outputColor = color;\n"
        "}";

const char* solid_color_vs =
        "#version 330\n"
        "\n"
        "layout(location = 0) in vec4 position;\n"
        "uniform mat4 mvp;\n"
        "\n"
        "void main() {\n"
        "  gl_Position = mvp * position;\n"
        "}";

const char* any_color_fs =
        "#version 330\n"
        "\n"
        "in vec4 vertex_color;\n"
        "\n"
        "out vec4 outputColor;\n"
        "\n"
        "void main() {\n"
        "  outputColor = vertex_color;\n"
        "}";

const char* any_color_vs =
        "#version 330\n"
        "\n"
        "layout(location = 0) in vec4 position;\n"
        "layout(location = 1) in vec4 color;\n"
        "\n"
        "uniform mat4 mvp;\n"
        "\n"
        "out vec4 vertex_color;\n"
        "\n"
        "void main() {\n"
        "  gl_Position = mvp * position;\n"
        "  vertex_color = color;\n"
        "}";

void RaceWindow::initOpenGl()
{
    simulationArea.make_current();
    simulationArea.throw_if_error();

    glewInit();

    coneModel = std::make_unique<GlModel>(CONE_MODEL_PATH, true);
    carModel = std::make_unique<GlModel>(CAR_MODEL_PATH);

    cone_program = std::make_unique<Program>(std::initializer_list<Shader>(
            {Shader(GL_VERTEX_SHADER, cone_vs), Shader(GL_FRAGMENT_SHADER, cone_fs)}));
    solid_color_program = std::make_unique<Program>(std::initializer_list<Shader>(
            {Shader(GL_VERTEX_SHADER, solid_color_vs), Shader(GL_FRAGMENT_SHADER, solid_color_fs)}));
    any_color_program = std::make_unique<Program>(std::initializer_list<Shader>(
            {Shader(GL_VERTEX_SHADER, any_color_vs), Shader(GL_FRAGMENT_SHADER, any_color_fs)}));

    cone_program->get_shader_uniforms(
            "mvp", mvpUniform, "coneColor", coneColorUniform,
            "lightDirection", lightDirectionUniform, "stripeStart1", stripeStart1Uniform,
            "stripeStart2", stripeStart2Uniform, "stripeEnd1", stripeEnd1Uniform,
            "stripeEnd2", stripeEnd2Uniform, "stripeAngle", stripeAngleUniform, "texture", textureUniform);
    solid_color_program->get_shader_uniforms("mvp", solidMvpUniform, "color", solidColorUniform);
    any_color_program->get_shader_uniforms("mvp", anyMvpUniform);

    RgbImage cone_image(CONE_STRIPE_IMAGE_PATH);
    black_cone_texture = cone_image.create_texture(GL_REPEAT);
    //invert the colors for black on white texture
    for (int i = 0; i < cone_image.size(); ++i) {
        Pixel* p = &cone_image.get_buffer()[i];
        p->r = static_cast<uint8_t>(255 - p->r);
        p->g = static_cast<uint8_t>(255 - p->g);
        p->b = static_cast<uint8_t>(255 - p->b);
    }
    white_cone_texture = cone_image.create_texture(GL_REPEAT);
}

void RaceWindow::deinitOpenGl()
{
    simulationArea.make_current();
    simulationArea.throw_if_error();

    coneModel.reset();
    carModel.reset();
    cone_program.reset();
    solid_color_program.reset();
    any_color_program.reset();

    glDeleteTextures(1, &black_cone_texture);
    glDeleteTextures(1, &white_cone_texture);
}

bool check_bounding_box_visible(glm::vec4& min, glm::vec4& max){
    return min[0] > -1.f && min[1] > -1.f && max[0] < 1.f && max[1] < 1.f //On screen
           && max[2] < 1.f //Depth ok
           && max[0] - min[0] > 0.005 && max[1] - min[1] > 0.005; //Big enough
}

bool RaceWindow::render(const Glib::RefPtr<Gdk::GLContext>& /* context */)
{
    if(key_pressed(GDK_KEY_p)){
        if(!lastPausePress) {
            lastPausePress = true;
            pause = !pause;
        }
    }
    else{
        lastPausePress = false;
    }

    if(key_pressed(GDK_KEY_equal))
        zoom *= 1.01;
    if(key_pressed(GDK_KEY_minus))
        zoom /= 1.01;

    glClearColor(.5, 0.5, 0.5, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glClearDepth(1.f);
    glClear(GL_DEPTH_BUFFER_BIT);


    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    using namespace std::placeholders;
    std::function<bool(guint)> pressed = std::bind(&RaceWindow::key_pressed, this, _1);

    glm::mat4 rotation = glm::rotate(glm::rotate(glm::rotate(glm::mat4(1.f),
                                                             glm::radians(viewAngles[2]),
                                                             glm::vec3(0.f, 0.f, 1.f)),
                                                 glm::radians(viewAngles[1]), glm::vec3(0.f, 1.f, 0.f)),
                                     glm::radians(viewAngles[0]), glm::vec3(1.f, 0.f, 0.f));

    float aspect_ratio = static_cast<float>(simulationArea.get_width()) / simulationArea.get_height();
    //TODO note that the autonomous cameras only have a 56 degree fov
    glm::mat4 perspective = glm::perspective(glm::radians(120.f) / aspect_ratio, aspect_ratio, .1f, 200.f);
    glm::mat4 look_at = glm::lookAt(glm::vec3(0.45f, 1.f, 0), glm::vec3(1.45f, .8f, 0.f), glm::vec3(0.f, 1.f, 0.f));
    glm::mat4 ortho = glm::scale(

            glm::ortho(-25.f, 25.f, -25.f / aspect_ratio, 25.f / aspect_ratio, -25.f, 25.f) *
            glm::rotate(glm::mat4(1.f), PI/2, glm::vec3(1.f, 0, 0)),
            glm::vec3(zoom));
    glm::mat4 camera = key_pressed(GDK_KEY_space) ? rotation * ortho : perspective * rotation * look_at;

    glm::mat4 car_transform = _simulation->get_transform();
    std::vector<std::pair<glm::vec4, glm::vec4>> cone_min_max_before;
    if(SAVE_CONES){
        for(auto cone : cones) {
            glm::mat4 cone_transform = glm::inverse(car_transform) *
                                       glm::rotate(glm::translate(glm::mat4(1.f), glm::vec3(cone.x, 0, cone.z)),
                                                   cone.angle,
                                                   glm::vec3(0.f, 1.f, 0.f));
            glm::mat4 mvp = camera * cone_transform;
            glm::vec4 min(1.f), max(-1.f);

            for (auto vertex : coneModel->get_vertices()) {
                glm::vec4 transformed(mvp * glm::vec4(vertex.position, 1.f));
                transformed = transformed / transformed[3];
                min = glm::min(min, transformed);
                max = glm::max(max, transformed);
            }
            cone_min_max_before.emplace_back(min, max);
        }
    }

    glUseProgram(any_color_program->program());
    glClear(GL_DEPTH_BUFFER_BIT);
    _simulation->draw_info(camera * glm::inverse(car_transform), anyMvpUniform, pause);
    glUseProgram(0);

    //This method works for motion blur, but isn't ideal.
    //TODO The quick and better way is to get rid of the alpha channel and average several images using another buffer
    //TODO Faster and possibly better (more points), but possibly less accurate (assumes linear) is here:
    //TODO https://developer.nvidia.com/gpugems/GPUGems3/gpugems3_ch27.html
    constexpr unsigned motionBlurCount = 25;
    for (int i = 0; i < motionBlurCount; ++i) {
        if(!pause)
            _simulation->update(pressed, _mouse, 1/60.f/motionBlurCount);
        glClear(GL_DEPTH_BUFFER_BIT);
        car_transform = _simulation->get_transform();
        drawWorld(camera * glm::inverse(car_transform));
    }

    car_transform = _simulation->get_transform();
    drawNoBlur(camera * glm::inverse(car_transform));

    static float t = 0;
    char time[20];
    if(!pause)
        t+=1/60.f;
    int length = snprintf(time, sizeof(time), "Time: %f", t);
    timeTextBuffer->set_text(time, &time[length]);


    std::vector<glm::vec4> bounding_boxes;
    std::vector<std::tuple<int, glm::vec4, glm::vec4>> labels_2d;
    std::vector<glm::vec3> positions_3d;

    if(SAVE_CONES) {
        for (int k = 0; k < cones.size(); ++k) {
            Cone cone = cones[k];
            glm::mat4 cone_transform = glm::inverse(car_transform) *
                                       glm::rotate(glm::translate(glm::mat4(1.f), glm::vec3(cone.x, 0, cone.z)),
                                                   cone.angle,
                                                   glm::vec3(0.f, 1.f, 0.f));
            glm::mat4 mvp = camera * cone_transform;
            glm::vec4 min = cone_min_max_before[k].first;
            glm::vec4 max = cone_min_max_before[k].second;

            if (!check_bounding_box_visible(min, max))
                continue;

            for (auto vertex : coneModel->get_vertices()) {
                glm::vec4 transformed(mvp * glm::vec4(vertex.position, 1.f));
                transformed = transformed / transformed[3];
                min = glm::min(min, transformed);
                max = glm::max(max, transformed);
            }

            if (check_bounding_box_visible(min, max)) {
                bounding_boxes.push_back(min);
                bounding_boxes.emplace_back(min[0], max[1], min[2], min[3]);
                bounding_boxes.emplace_back(min[0], max[1], min[2], min[3]);
                bounding_boxes.push_back(max);
                bounding_boxes.push_back(max);
                bounding_boxes.emplace_back(max[0], min[1], min[2], min[3]);
                bounding_boxes.emplace_back(max[0], min[1], min[2], min[3]);
                bounding_boxes.push_back(min);

                bounding_boxes.push_back(min);
                bounding_boxes.emplace_back(min[0], max[1], max[2], max[3]);
                bounding_boxes.emplace_back(min[0], max[1], max[2], max[3]);
                bounding_boxes.push_back(max);
                bounding_boxes.push_back(max);
                bounding_boxes.emplace_back(max[0], min[1], max[2], max[3]);
                bounding_boxes.emplace_back(max[0], min[1], max[2], max[3]);
                bounding_boxes.push_back(min);

                labels_2d.emplace_back(static_cast<int>(cone.type), min, max);
                positions_3d.emplace_back(glm::inverse(car_transform) * glm::vec4(cone.x, 0, cone.z, 1.f));
            }
        }
    }

    if(DRAW_DEBUG){
        glUseProgram(solid_color_program->program());
        glClear(GL_DEPTH_BUFFER_BIT);
        glm::mat4 mvp(1.f);
        drawLines(bounding_boxes, mvp);
        glUseProgram(0);
    }
    static int j = 1000;
    if(j < 1000 && !pause && SAVE_CONES){
        j++;
        char path[100]={};
        sprintf(path, IMAGE_OUTPUT_PATH, j);
        RgbImage(0, 0, simulationArea.get_width(), simulationArea.get_height()).save_ppm(path);

        sprintf(path, LABEL2D_OUTPUT_PATH, j);
        std::cout << path << std::endl;
        std::ofstream label_file(path, std::ios::trunc);
        for(auto& label : labels_2d){
            auto min = std::get<1>(label);
            auto max = std::get<2>(label);
            label_file << std::get<0>(label) << ' '
                       << (min[0]+max[0]) / 4.f + .5f << ' '
                       << (-min[1] - max[1])/4.f + .5f << ' '
                       << (max[0]-min[0]) / 2.f << ' '
                       << (max[1]-min[1]) / 2.f << std::endl;
        }

        sprintf(path, LABEL3D_OUTPUT_PATH, j);
        std::ofstream label3d_file(path, std::ios::trunc);

        glm::vec3 car_position;
        glm::vec3 car_velocity;
        glm::vec3 car_orientation;
        glm::vec3 car_angular_velocity;
        _simulation->get_state(car_position, car_velocity, car_orientation, car_angular_velocity);
        car_velocity = glm::rotate(glm::mat4(1.f), car_orientation[1], glm::vec3(0, 1, 0)) * glm::vec4(car_velocity, 1);
        label3d_file << "Angular Velocity, X Velocity, Y Velocity" << std::endl;
        label3d_file << car_angular_velocity[1] << ' '
                     << car_velocity[0] << ' '
                     << car_velocity[2] << ' '
                     //<< pitch << ' '
                     //<< roll << ' '
                     << std::endl << std::endl;

        label3d_file << "X Position, Y Position, Z position" << std::endl;
        for(auto& position : positions_3d){
            label3d_file << position[0] << ' '
                         << -position[2] << ' '
                         << position[1] << ' '
                         << std::endl;
        }
    }

    glFlush();

    //std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() << endl;

    simulationArea.throw_if_error();

    return true;
}

void RaceWindow::drawWorld(const glm::mat4& camera)
{
    glm::vec4 global_light_direction(.2, .7, 0, 0);

    glUseProgram(cone_program->program());

    glUniform1f(stripeStart1Uniform, .54f / 1.5f * .325f);
    glUniform1f(stripeEnd1Uniform, 1 / 1.5f * .325f);
    glUniform1f(stripeStart2Uniform, 1.f);
    glUniform1f(stripeEnd2Uniform, 1.f);

    glUniform1i(textureUniform, 0);
    for (auto cone : cones) {
        glm::mat4 model =
                glm::rotate(glm::translate(glm::mat4(1.f), glm::vec3(cone.x, 0, cone.z)), cone.angle,
                            glm::vec3(0.f, 1.f, 0.f));
        glm::mat4 mvp = camera * model;
        glUniformMatrix4fv(mvpUniform, 1, GL_FALSE, reinterpret_cast<float *>(&mvp));

        glm::vec3 cone_light_direction = glm::inverse(model) * global_light_direction;
        glUniform3fv(lightDirectionUniform, 1, reinterpret_cast<float *>(&cone_light_direction));

        glm::vec4 cone_color;
        GLuint texture;
        switch (cone.type) {
            case Cone::Type::Blue:
                cone_color = glm::vec4(57.f / 255, 80.f / 255, 165.f / 255, .1);
                texture = white_cone_texture;
                break;
            case Cone::Type::Yellow:
                cone_color = glm::vec4(226.f / 255, 181.f / 255, 38.f / 255, .1);
                texture = black_cone_texture;
                break;
            case Cone::Type::Orange:
                cone_color = glm::vec4(207.f / 255, 54.f / 255, 38.f / 255, .1);
                texture = white_cone_texture;
                break;
            case Cone::Type::TallOrange:
                cone_color = glm::vec4(207.f / 255, 54.f / 255, 38.f / 255, .1);
                texture = white_cone_texture;
                break;
        }

        glUniform4fv(coneColorUniform, 1, reinterpret_cast<float *>(&cone_color));
        glUniform1f(stripeAngleUniform, cone.stripeAngle);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);
        coneModel->draw();
    }
}

void RaceWindow::drawNoBlur(const glm::mat4 &camera){

    glm::mat4 car_transform = _simulation->get_transform();

    glm::mat4 model = glm::rotate(
            glm::mat4(1.f), PI/2, glm::vec3(1, 0, 0));
    glm::mat4 mvp = camera * car_transform * model;
    glUniformMatrix4fv(mvpUniform, 1, GL_FALSE, reinterpret_cast<float *>(&mvp));

    glm::vec3 light_direction = glm::inverse(car_transform) * glm::vec4(.2, .7, 0, 0);
    glUniform3fv(lightDirectionUniform, 1, reinterpret_cast<float *>(&light_direction));

    glUniform1f(stripeStart1Uniform, 5.f);
    glUniform1f(stripeEnd1Uniform, 5.f);
    glUniform1f(stripeStart2Uniform, 5.f);
    glUniform1f(stripeEnd2Uniform, 5.f);
    glm::vec4 car_color(0.3f, .3f, .3f, 1.f);
    glUniform4fv(coneColorUniform, 1, reinterpret_cast<float *>(&car_color));
    glUniform1f(stripeAngleUniform, 0);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, black_cone_texture);
    carModel->draw();

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    if(DRAW_DEBUG){
        glUseProgram(solid_color_program->program());
        glClear(GL_DEPTH_BUFFER_BIT);
        _simulation->draw_debug(camera, solidMvpUniform, solidColorUniform);
        glUseProgram(0);
    }
}

void RaceWindow::updateFromAdjustment(float *dest, const Glib::RefPtr<Gtk::Adjustment> &a)
{
    *dest = static_cast<float>(a->get_value());
}

bool RaceWindow::on_key_press(GdkEventKey *event) {
    _keyboard_map[event->keyval] = event->type == GDK_KEY_PRESS;
    return true;
}

bool RaceWindow::key_pressed(guint key) {
    auto it = _keyboard_map.find(key);
    if(it == _keyboard_map.end())
        return false;
    return it->second;
}

gboolean RaceWindow::on_mouse_move(GdkEventMotion *event) {
    _mouse.x = static_cast<float>((event->x - simulationArea.get_allocation().get_x()) * 2 / simulationArea.get_width() - 1);
    _mouse.y = static_cast<float>((event->y - simulationArea.get_allocation().get_y()) * 2 / simulationArea.get_height() - 1);
    //std::cout << _mouse.x << ',' <<_mouse.y << std::endl;
    return 0;
}

gboolean RaceWindow::on_mouse_scroll(GdkEventScroll *event){
    //Doesn't get called unless widget supports scrolling and haven't figured out how to enable, so commented out for now
    //This might help https://developer.gnome.org/gtk3/stable/GtkScrollable.html
    //if(event->direction == GDK_SCROLL_UP){
    //    _mouse.scroll++;
    //}
    //else if(event->direction == GDK_SCROLL_DOWN){
    //    _mouse.scroll--;
    //}
    //else if(event->direction == GDK_SCROLL_SMOOTH){
    //    _mouse.scroll += event->delta_y;
    //    //TODO see if same as gdk_event_get_scroll_deltas()
    //}
    //std::cout << 'A' << _mouse.scroll << std::endl;
    return false;
}

void RaceWindow::drawLines(const std::vector<glm::vec4> &vertices, glm::mat4 &mvp) {
    glUniformMatrix4fv(solidMvpUniform, 1, GL_FALSE, reinterpret_cast<float *>(&mvp));
    glm::vec4 red(1, 0, 0, 1);
    glUniform4fv(solidColorUniform, 1, reinterpret_cast<float *>(&red));

    glLineWidth(2);

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    GLuint vertex_buffer;
    glGenBuffers(1, &vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(vertices[0]), vertices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertices.size()));
    glDisableVertexAttribArray(0);

    glDeleteBuffers(1, &vertex_buffer);
    glDeleteVertexArrays(1, &vao);
}

gboolean RaceWindow::tick(GdkFrameClock *frame_clock) {
    //Force a repaint to make the simulation continue updating
    //TODO Consider disabling when paused
    simulationArea.queue_draw();
    return G_SOURCE_CONTINUE;
}

gboolean RaceWindow::staticTick(GtkWidget *, GdkFrameClock *frame_clock, gpointer user_data) {
    return reinterpret_cast<RaceWindow*>(user_data)->tick(frame_clock);
}