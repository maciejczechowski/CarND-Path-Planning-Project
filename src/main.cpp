#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "json.hpp"
#include "trajectory_generator.h"
#include "json_helpers.h"
#include "sensor_fusion.h"
#include "planner.h"
#include "map.h"
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors


    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    Map map;
    map.load_map(map_file_);
    // The max s value before wrapping around the track back to 0

    //starting lane

    double ref_velocity = 0; //mph

    Predictions predictions = Predictions();
    SensorFusion sensorFusion = SensorFusion(predictions);
    TrajectoryGenerator tgen = TrajectoryGenerator(map);
    Car car;

    Planner planner = Planner(tgen, sensorFusion, car);



    h.onMessage([&map, &car, &sensorFusion, &planner]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode) {


        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    car.x = j[1]["x"];
                    car.y = j[1]["y"];
                    car.s = j[1]["s"];
                    car.d = j[1]["d"];
                    car.yaw = j[1]["yaw"];
                    car.speed = j[1]["speed"];


                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    Trajectory previous = {previous_path_x, previous_path_y};

                    // Sensor Fusion Data, a list of all other cars on the same side
                    //   of the road.
                    vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
                    sensorFusion.setFusionData(sensor_fusion);

                    json msgJson;


                    /**
                     * TODO: define a path made up of (x,y) points that the car will visit
                     *   sequentially every .02 seconds
                     */


                    auto nextTrajectory = planner.Execute(previous);

                    msgJson["next_x"] = nextTrajectory.x_values;
                    msgJson["next_y"] = nextTrajectory.y_values;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}