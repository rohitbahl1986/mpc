/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "MPC.h"
#include "globalConstants.hpp"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include <chrono>
#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <thread>
#include <vector>

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using namespace std;
using namespace Eigen;
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
    return M_PI;
}

static double deg2rad(double x)
{
    return x * pi() / 180;
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial.
static double polyeval(VectorXd coeffs, double x)
{
    double result = 0.0;
    for (uint32_t i = 0; i < coeffs.size(); ++i)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(VectorXd xvals, VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

int main()
{
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
            uWS::OpCode opCode)
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
            string sdata = string(data).substr(0, length);
            if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
            {
                string s = hasData(sdata);
                if (s != "")
                {
                    auto j = json::parse(s);
                    string event = j[0].get<string>();
                    if (event == "telemetry")
                    {
                        // j[1] is the data JSON object
                        vector<double> ptsx = j[1]["ptsx"];
                        vector<double> ptsy = j[1]["ptsy"];
                        double px = j[1]["x"];
                        double py = j[1]["y"];
                        double psi = j[1]["psi"];
                        double v = j[1]["speed"];

                        VectorXd waypointx_car(ptsx.size());
                        VectorXd waypointy_car(ptsy.size());
                        uint32_t ptx_size = ptsx.size();
                        // Waypoints from car's perspective
                        for (uint32_t i = 0; i < ptx_size; ++i)
                        {
                            double dx = ptsx[i] - px;
                            double dy = ptsy[i] - py;
                            waypointx_car[i] = dx * cos(-psi) - dy * sin(-psi);
                            waypointy_car[i] = dx * sin(-psi) + dy * cos(-psi);
                        }

                        // Fits a 3rd-order polynomial to the above x and y coordinates
                        auto coeffs = polyfit(waypointx_car, waypointy_car, 3);
                        double epsi = -atan(coeffs[1]);
                        double cte = polyeval(coeffs, 0);

                        // latency
                        const double act_delay = 0.1;
                        const double delta = j[1]["steering_angle"];
                        const double a = j[1]["throttle"];

                        // prediction based on latency
                        const double x_with_delay = v * cos(0) * act_delay;
                        const double y_with_delay = v * sin(0) * act_delay;
                        const double psi_with_delay = -v * delta / Lf * act_delay;
                        const double v_with_delay = v + a * act_delay;
                        const double cte_with_delay = cte + v * sin(epsi) * act_delay;
                        const double epsi_with_delay = epsi - v * delta / Lf * act_delay;

                        VectorXd state(6);
                        state << x_with_delay, y_with_delay, psi_with_delay, v_with_delay, cte_with_delay, epsi_with_delay;

                        // Solve for new actuation (and to show predicted x and y in the future)
                        auto vars = mpc.Solve(state, coeffs);

                        double steer_value = vars[0] / (deg2rad(25) * Lf);
                        double throttle_value = vars[1];

                        // Send values to the simulator
                        json msgJson;
                        msgJson["steering_angle"] = steer_value;
                        msgJson["throttle"] = throttle_value;

                        vector<double> mpc_x_vals;
                        vector<double> mpc_y_vals;
                        uint32_t v_size = vars.size();
                        for (uint32_t i = 2; i < v_size; ++i)
                        {
                            mpc_x_vals.push_back(vars[i]);
                            mpc_y_vals.push_back(vars[++i]);
                        }

                        msgJson["mpc_x"] = mpc_x_vals;
                        msgJson["mpc_y"] = mpc_y_vals;

                        vector<double> next_x_vals;
                        vector<double> next_y_vals;
                        const double step = 2;

                        for (uint32_t i = 1; i < 30; ++i)
                        {
                            next_x_vals.push_back(step * i);
                            next_y_vals.push_back(polyeval(coeffs, step * i));
                        }

                        msgJson["next_x"] = next_x_vals;
                        msgJson["next_y"] = next_y_vals;

                        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                        this_thread::sleep_for(chrono::milliseconds(100));
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
                }
                else
                {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
        });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
            size_t, size_t)
    {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
            char *message, size_t length)
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
