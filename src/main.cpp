#include <math.h>
#include <iostream>
#include <uWS/uWS.h>

#include "json.hpp"

using json = nlohmann::json;

const int MAX_CONNECTIONS = 100;
const int FOOD_MAX = 100;
const int X_MAX = 1920;
const int Y_MAX = 1080;
const int REPEAT_MS = 40;
const double START_SIZE = 10.;

struct Vector {
  double x;
  double y;

  Vector() : x(0.), y(0.) {}
  Vector(double x, double y) : x(x), y(y) { }
  Vector(const Vector& v) : x(v.x), y(v.y) { }
  static Vector Random() { return Vector(std::rand() % X_MAX, std::rand() % Y_MAX); }

  double norm() {
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
  }

  Vector& operator+=(const Vector& v) { x += v.x; y += v.y; return *this; }
};

Vector operator*(const Vector& v, double f) { return Vector(v.x * f, v.y * f); }
Vector operator/(const Vector& v, double f) { return Vector(v.x / f, v.y / f); }
Vector operator-(const Vector& l, const Vector& r) { return Vector(l.x - r.x, l.y - r.y); }

void to_json(json &j, const Vector v) {
  j = json{v.x, v.y};
}

struct State {
  int id;
  double s;
  Vector p;
  Vector t;
  bool dead = false;
  bool spectating = false;

  State(int id, const Vector &p) : id(id), p(p), t(p), s(START_SIZE) {};

  void update(int dt) {
    Vector pt = (t - p);
    double pt_norm = pt.norm();

    double l = (dt / 5.) / (std::sqrt(s / START_SIZE));
    if (l < pt_norm) {
      p += (pt * l) / pt_norm;
    } else {
      p = t;
    }

    p.x = std::max(p.x, 0.);
    p.x = std::min(p.x, (double) X_MAX);
    p.y = std::max(p.y, 0.);
    p.y = std::min(p.y, (double) Y_MAX);
    s = std::min(s, (double) (X_MAX / 4));
  }

  void set_target(int x, int y) { this->t.x = x; this->t.y = y; }
  void reset() { p = Vector::Random(); s = START_SIZE; t = p; }
  bool is_eating(const Vector &v) { return (p - v).norm() < s; }
  bool is_eating(const State *st) { return ((p - st->p).norm() < s + st->s) && s > st->s; }
  void eat() { s = std::sqrt(std::pow(s, 2) + 5); }
  void eat(const State *st) { s = std::sqrt(std::pow(s, 2) + std::pow(st->s, 2)); }
  bool is_playing() { return !dead && !spectating; }
};

void to_json(json &j, const State *s) {
  j = json{s->id, s->p.x, s->p.y, s->s};
}

int milliseconds_since_epoch() {
  return  std::chrono::duration_cast<std::chrono::milliseconds>
    (std::chrono::system_clock::now().time_since_epoch()).count();
}

int main() {
  uWS::Hub hub;
  int next_id = 0;
  int connections = 0;

  std::vector<Vector> foods(FOOD_MAX);
  std::generate(foods.begin(), foods.end(), []() { return Vector::Random(); });

  hub.onConnection([&next_id, &connections](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
    int id = next_id++;
    connections++;

    State* state = new State(id, Vector::Random());
    ws->setUserData(state);
    if(connections > MAX_CONNECTIONS) { state->spectating = true; return; };

    json j = {{"id", id}};
    std::vector<std::uint8_t> jcbor = json::to_cbor(j);
    ws->send(reinterpret_cast<char*>(jcbor.data()), jcbor.size(), uWS::BINARY);
  });

  hub.onMessage([](uWS::WebSocket<uWS::SERVER> *ws, char *message, size_t length, uWS::OpCode opCode) {
    State *state = (State *) ws->getUserData();
    if (!state->is_playing()) { return; }

    try {
      std::string message_s(message, length);
      json message_j = json::parse(message_s);
  
      if (message_j["x"].is_number_integer() && message_j["y"].is_number_integer()) {
        state->set_target((int) message_j["x"], (int) message_j["y"]);
      } 
    } catch(std::exception) {
      ws->terminate();
    }
  });

  hub.onDisconnection([&connections](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
    State *state = (State*) ws->getUserData();
    connections--;
    delete state;
  });

  struct TimerData { uWS::Hub* hub; std::vector<Vector> *foods; int *last_timestamp; };
  TimerData timer_data = { &hub, &foods, new int };

  Timer *timer = new Timer(hub.getLoop());
  timer->setData((void*) &timer_data);
  timer->start([](Timer *timer) {
    TimerData *timer_data = (TimerData*)timer->getData();
    uWS::Hub *hub = timer_data->hub;
    std::vector<Vector> *foods = timer_data->foods;

    int timestamp = milliseconds_since_epoch();
    int *last_timestamp = timer_data->last_timestamp;
    int delta_timestamp = last_timestamp ? timestamp - (*last_timestamp) : 0;
    *last_timestamp = milliseconds_since_epoch();

    hub->getDefaultGroup<uWS::SERVER>().forEach([hub, delta_timestamp, foods](uWS::WebSocket<uWS::SERVER> *ws) {
      State *state = (State *) ws->getUserData();
      if (!state->is_playing()) { return; }

      state->update(delta_timestamp);

      for (auto food_it = foods->begin(); food_it != foods->end(); ++food_it) {
        if (state->is_eating(*food_it)) {
          (*food_it) = Vector::Random();
          state->eat();
        }
      }

      hub->getDefaultGroup<uWS::SERVER>().forEach([state](uWS::WebSocket<uWS::SERVER> *ws_opponent) {
        State *state_opponent = (State *) ws_opponent->getUserData();
        if (!state_opponent->is_playing()) { return; }

        if (state->is_eating(state_opponent)) {
          state->eat(state_opponent);
          state_opponent->reset();
          state_opponent->dead = true;
        }
      });
    });

    std::vector<State*> states;
    hub->getDefaultGroup<uWS::SERVER>().forEach([&states](uWS::WebSocket<uWS::SERVER> *ws) {
      State *state = (State *) ws->getUserData();
      if (!state->is_playing()) {
        state->dead = false;
        return;
      }
      states.push_back(state);
    });

    json j = {{"timestamp", timestamp}, {"states", states}, {"foods", *foods}};
    std::vector<std::uint8_t> jcbor = json::to_cbor(j);
    hub->getDefaultGroup<uWS::SERVER>().broadcast(reinterpret_cast<char*>(jcbor.data()), jcbor.size(), uWS::BINARY);
  }, 0, REPEAT_MS);

  hub.listen(4001);
  hub.run();
}
