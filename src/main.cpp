// main.cpp - окно SFML 3, главный цикл (аккумулятор + интерполяция), отрисовка, HUD, клавиши.
#include <SFML/Graphics.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <initializer_list>
#include <optional>
#include <string>
#include <vector>

#include "Experiments.h"
#include "Scenes.h"

namespace {

using Clock = std::chrono::steady_clock;

// Усреднённые за HudPeriod значения (в одном кадре шагов может быть 0, поэтому берём среднее)
struct Hud {
    double t = 0.0;
    int frames = 0;
    double physMs = 0.0;
    long steps = 0, fastSteps = 0, contacts = 0;
    // что показываем сейчас
    double fps = 0, msFrame = 0, stepsFrame = 0, fastFrame = 0, contactsStep = 0;
};

struct Counter {  // накопитель статистики шагов
    StepStats total;
    void add(const StepStats& s) {
        total.contacts += s.contacts; total.ccdIterations += s.ccdIterations;
        total.ccdLimitHits += s.ccdLimitHits; total.slowSteps += s.slowSteps; total.fastSteps += s.fastSteps;
    }
};

std::string f2(double v, int prec = 2) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
    return buf;
}

} // namespace

int main(int argc, char** argv) {
    for (const std::string& w : cfg::validate()) std::printf("CONFIG WARNING: %s\n", w.c_str());
    if (argc > 1 && std::string(argv[1]) == "--experiments") {
        runAllExperiments("experiments.txt");
        return 0;
    }

    sf::RenderWindow window(sf::VideoMode({cfg::WindowW, cfg::WindowH}), "PZ3: CCD vs multi-rate (C++, SFML 3)");
    window.setVerticalSyncEnabled(cfg::VSync);

    // шрифт: сначала assets/font.ttf рядом с программой, потом системные
    sf::Font font;
    bool fontOk = false;
    for (const char* path : {"assets/font.ttf", "C:/Windows/Fonts/consola.ttf", "C:/Windows/Fonts/arial.ttf",
                             "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"}) {
        if (font.openFromFile(path)) { fontOk = true; break; }
    }
    if (!fontOk) std::printf("WARNING: no font found, HUD text will not be drawn (put assets/font.ttf next to the exe)\n");

    // ---------- состояние ----------
    SceneId scene = SceneId::Tunnel;
    Mode mode = Mode::Discrete;
    int hz = cfg::DefaultHz;
    bool paused = false, stepOnce = false, interp = true, showHelp = false;
    SceneParams params;
    World world;
    buildScene(world, scene, params);
    double acc = 0.0;
    Hud hud;
    Counter cnt;
    int shotIndex = 0;
    std::string message;           // короткое сообщение внизу
    double messageTime = 0.0;

    auto reset = [&]() { buildScene(world, scene, params); acc = 0.0; };
    auto setScene = [&](SceneId id) { scene = id; reset(); };
    auto setHz = [&](int f) { hz = f; acc = 0.0; };
    auto say = [&](const std::string& s) { message = s; messageTime = 4.0; };

    sf::CircleShape circle;
    sf::RectangleShape rect;
    sf::Text text(font, "", 16);
    sf::Clock frameClock;

    while (window.isOpen()) {
        // ---------- события ----------
        bool runExperiments = false;
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            } else if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
                switch (key->code) {
                    case sf::Keyboard::Key::Escape: window.close(); break;
                    // сцены
                    case sf::Keyboard::Key::Num1: setScene(SceneId::Room); break;
                    case sf::Keyboard::Key::Num2: setScene(SceneId::Tunnel); break;
                    case sf::Keyboard::Key::Num3: setScene(SceneId::MultiRate); break;
                    case sf::Keyboard::Key::S:    setScene(SceneId::Stack); break;
                    // режимы
                    case sf::Keyboard::Key::D: mode = Mode::Discrete;  reset(); break;
                    case sf::Keyboard::Key::C: mode = Mode::CCD;       reset(); break;
                    case sf::Keyboard::Key::M: mode = Mode::MultiRate; reset(); break;
                    // частота физики
                    case sf::Keyboard::Key::Num4: setHz(cfg::FreqPresets[0]); break;
                    case sf::Keyboard::Key::Num5: setHz(cfg::FreqPresets[1]); break;
                    case sf::Keyboard::Key::Num6: setHz(cfg::FreqPresets[2]); break;
                    // управление
                    case sf::Keyboard::Key::P: paused = !paused; break;
                    case sf::Keyboard::Key::N: if (paused) stepOnce = true; break;
                    case sf::Keyboard::Key::I: interp = !interp; break;
                    case sf::Keyboard::Key::R: reset(); break;
                    case sf::Keyboard::Key::H: showHelp = !showHelp; break;
                    case sf::Keyboard::Key::T: runExperiments = true; break;
                    // e и скорость снаряда
                    case sf::Keyboard::Key::Up:
                        params.restitution = std::min(1.f, params.restitution + 0.1f);
                        world.restitution = params.restitution; break;
                    case sf::Keyboard::Key::Down:
                        params.restitution = std::max(0.f, params.restitution - 0.1f);
                        world.restitution = params.restitution; break;
                    case sf::Keyboard::Key::Right:
                        params.projectileSpeed += 500.f; reset(); break;
                    case sf::Keyboard::Key::Left:
                        params.projectileSpeed = std::max(500.f, params.projectileSpeed - 500.f); reset(); break;
                    case sf::Keyboard::Key::F12: {
                        sf::Texture tex;
                        std::string name = "screenshot_" + std::to_string(++shotIndex) + ".png";
                        if (tex.resize(window.getSize())) {
                            tex.update(window);
                            if (tex.copyToImage().saveToFile(name)) say("saved " + name);
                            else say("could not save " + name);
                        }
                        break;
                    }
                    default: break;
                }
            }
        }

        double ft = std::min<double>(frameClock.restart().asSeconds(), cfg::MaxFrameTime);

        if (runExperiments) {        // эксперименты без отрисовки; результат в консоль и experiments.txt
            window.clear(sf::Color(24, 26, 32));
            if (fontOk) { text.setString("Running experiments... see console"); text.setPosition({20.f, 20.f});
                          text.setFillColor(sf::Color::White); window.draw(text); }
            window.display();
            runAllExperiments("experiments.txt");
            say("experiments done -> experiments.txt");
            frameClock.restart();
            continue;
        }

        // ---------- физика: фиксированный шаг через аккумулятор ----------
        const double dt = 1.0 / hz;
        if (!paused) acc += ft;
        int steps = 0;
        StepStats frameStats;
        auto t0 = Clock::now();
        while (!paused && acc >= dt) {
            stepWorld(world, mode, (float)dt, frameStats);
            acc -= dt;
            ++steps;
        }
        if (paused && stepOnce) {
            stepWorld(world, mode, (float)dt, frameStats);
            ++steps;
            stepOnce = false;
        }
        double physMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        cnt.add(frameStats);

        // ---------- HUD: среднее за полсекунды ----------
        hud.t += ft; hud.frames++; hud.physMs += physMs;
        hud.steps += frameStats.slowSteps; hud.fastSteps += frameStats.fastSteps; hud.contacts += frameStats.contacts;
        if (hud.t >= cfg::HudPeriod) {
            hud.fps = hud.frames / hud.t;
            hud.msFrame = hud.physMs / hud.frames;
            hud.stepsFrame = (double)hud.steps / hud.frames;
            hud.fastFrame = (double)hud.fastSteps / hud.frames;
            hud.contactsStep = hud.steps ? (double)hud.contacts / hud.steps : 0.0;
            std::printf("%s | %s | %d Hz | FPS %.0f | %.3f ms/frame | steps/frame %.2f | fast steps/frame %.2f | contacts/step %.1f | tunneled %d/%d\n",
                        sceneName(scene), modeName(mode), hz, hud.fps, hud.msFrame, hud.stepsFrame, hud.fastFrame,
                        hud.contactsStep, world.tunneledCount(), (int)world.fast.size());
            hud.t = 0; hud.frames = 0; hud.physMs = 0; hud.steps = 0; hud.fastSteps = 0; hud.contacts = 0;
        }
        if (messageTime > 0.0) messageTime -= ft;

        // ---------- отрисовка ----------
        const float alpha = (paused || !interp) ? 1.f : (float)(acc / dt);
        auto pos = [&](const Body& b) { return interp ? lerp(b.prevFrame, b.pos, alpha) : b.pos; };

        window.clear(sf::Color(21, 31, 11));

        // рамка комнаты (сами стены комнаты лежат снаружи окна)
        rect.setFillColor(sf::Color(80, 85, 100));
        const float W = (float)cfg::WindowW, H = (float)cfg::WindowH;
        auto edge = [&](float x, float y, float w, float h) {
            rect.setPosition({x, y}); rect.setSize({w, h}); window.draw(rect);
        };
        edge(0.f, 0.f, W, 3.f); edge(0.f, H - 3.f, W, 3.f);
        edge(0.f, 0.f, 3.f, H); edge(W - 3.f, 0.f, 3.f, H);
        // стенки вне комнаты (тонкая стенка, упоры стека)
        for (std::size_t i = 4; i < world.walls.size(); ++i) {
            const Wall& wl = world.walls[i];
            rect.setPosition({wl.min.x, wl.min.y});
            rect.setSize({wl.max.x - wl.min.x, wl.max.y - wl.min.y});
            rect.setFillColor(wl.thin ? sf::Color(235, 235, 235) : sf::Color(120, 125, 140));
            window.draw(rect);
        }
        auto drawBody = [&](const Body& b, sf::Color fill, sf::Color outline) {
            Vec2 p = pos(b);
            circle.setPointCount(b.radius > 12.f ? 40 : 20);
            circle.setRadius(b.radius);
            circle.setOrigin({b.radius, b.radius});
            circle.setPosition({p.x, p.y});
            circle.setFillColor(fill);
            circle.setOutlineThickness(1.f);
            circle.setOutlineColor(outline);
            window.draw(circle);
        };
        for (const Body& b : world.statics) drawBody(b, sf::Color(110, 115, 130), sf::Color(170, 175, 190));
        for (const Body& b : world.slow)    drawBody(b, sf::Color(70, 130, 255), sf::Color(20, 40, 110));
        for (const Body& b : world.fast) {
            sf::Color fill(255, 136, 0);                       // оранжевый: летит
            if (b.tunneled) fill = sf::Color(247, 5, 239);     // красный: пролетел сквозь стенку
            else if (b.hitThin) fill = sf::Color(37, 247, 5);// голубой: отбился от стенки
            drawBody(b, fill, sf::Color(90, 60, 0));
        }

        if (fontOk) {
            auto line = [&](const std::string& s, float x, float y, sf::Color c = sf::Color::White) {
                text.setString(s); text.setPosition({x, y}); text.setFillColor(c); window.draw(text);
            };
            rect.setFillColor(sf::Color(0, 0, 0, 160));
            rect.setPosition({0.f, 0.f});
            rect.setSize({W, 104.f});
            window.draw(rect);

            std::string l1 = std::string("Scene: ") + sceneName(scene) + " | Mode: " + modeName(mode) +
                             " | Physics: " + std::to_string(hz) + " Hz";
            if (mode == Mode::MultiRate) l1 += " | Fast: " + std::to_string(hz * cfg::FastSubstepsK) + " Hz (k=" +
                                               std::to_string(cfg::FastSubstepsK) + ")";
            std::string l2 = "FPS: " + f2(hud.fps, 0) + " | physics: " + f2(hud.msFrame, 3) +
                             " ms/frame | steps/frame: " + f2(hud.stepsFrame);
            if (mode == Mode::MultiRate) l2 += " | fast steps/frame: " + f2(hud.fastFrame);
            std::string l3 = "contacts/step: " + f2(hud.contactsStep, 1) + " | bodies: " +
                             std::to_string(world.statics.size()) + " + " + std::to_string(world.slow.size()) + " + " +
                             std::to_string(world.fast.size()) + " | e = " + f2(world.restitution) +
                             " | v = " + f2(params.projectileSpeed, 0) + " px/s";
            std::string l4;
            if (world.thinWallIndex >= 0)
                l4 = "Tunneled: " + std::to_string(world.tunneledCount()) + " / " + std::to_string(world.fast.size()) + "   ";
            l4 += std::string(interp ? "interp: on" : "interp: off") + (paused ? "   [PAUSED]" : "");
            line(l1, 10.f, 4.f);
            line(l2, 10.f, 26.f);
            line(l3, 10.f, 48.f);
            line(l4, 10.f, 70.f, world.tunneledCount() > 0 ? sf::Color(255, 120, 120) : sf::Color(160, 255, 160));
            line("H - help", 10.f, 86.f, sf::Color(150, 150, 150));

            if (messageTime > 0.0) line(message, 10.f, H - 28.f, sf::Color(255, 230, 120));
            if (showHelp) {
                static const std::vector<std::string> help = {
                    "1 Room  2 Tunnel  3 Multi-rate  S Stack",
                    "D Discrete  C CCD  M Multi-rate",
                    "4/5/6  physics 60/120/240 Hz",
                    "P pause  N one step (on pause)  R restart",
                    "I interpolation on/off",
                    "Up/Down  restitution e +-0.1",
                    "Left/Right  projectile speed -+500 px/s (restart)",
                    "T experiments (console + experiments.txt)",
                    "F12 screenshot   Esc exit"};
                rect.setFillColor(sf::Color(0, 0, 0, 200));
                rect.setPosition({W - 470.f, 112.f});
                rect.setSize({460.f, 12.f + 22.f * (float)help.size()});
                window.draw(rect);
                float y = 118.f;
                for (const std::string& s : help) { line(s, W - 460.f, y); y += 22.f; }
            }
        }
        window.display();
    }
    return 0;
}
