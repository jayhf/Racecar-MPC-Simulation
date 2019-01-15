#include "RaceWindow.hpp"

#include <gtkmm.h>

int main (int argc, char *argv[])
{
    //Just run the simulation with no graphics
    //auto start = std::chrono::high_resolution_clock::now();
    //auto sim = create_simulation();
    //std::function<bool(guint)> pressed = [](guint x) -> bool{return x%2 == 0;};
    //MouseInfo mouse{};
    //sim->update(pressed, mouse, 100.f);
    //auto finish = std::chrono::high_resolution_clock::now();
    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count()/1000.f << std::endl;


    auto app = Gtk::Application::create();
    RaceWindow window;
    return app->run(window, argc, argv);
}