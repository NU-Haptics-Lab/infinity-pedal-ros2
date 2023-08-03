/* infinity-pedal, a linux driver for the Infinity IN-USB-2 foot pedal
 * Copyright (C) 2012 Scott Squires
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <string.h>
#include "event.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher;



void event_callback(pedal_context *context, int event, void *client_data)
{
    auto msg = std_msgs::msg::Bool();
    switch(event)
    {
        case PEDAL_EVENT_UP_MIDDLE:
            msg.data = false;
            publisher->publish(msg);
            break;

        case PEDAL_EVENT_DOWN_MIDDLE:
            msg.data = true;
            publisher->publish(msg);
            break;
    }
}

int main(int argc, char *argv[])
{
    int result = 0;
    char cmd[1024];
    char buf[1024];
    FILE *mplayer_pipe = NULL;

    snprintf(cmd, 1024, "mplayer -slave");

    int i;
    for (i = 1; i < argc; i++)
    {
        strncpy(buf, cmd, 1024);
        /* TODO: escape parameter for the new shell call */
        /* (prefix any single quotes with \, and wrap whole parameter in single quotes) */
        snprintf(cmd, 1024, "%s %s", buf, argv[i]);
    }

    printf("%s\n", cmd);
    mplayer_pipe = popen(cmd, "w");

    pedal_context *context = pedal_new();
    pedal_subscribe(context, PEDAL_EVENT_UP_LEFT | PEDAL_EVENT_UP_MIDDLE | PEDAL_EVENT_UP_RIGHT);
    pedal_set_callback(context, &event_callback);
    pedal_set_client_data(context, mplayer_pipe);

    // ROS stuff
    // https://roboticsbackend.com/write-minimal-ros2-cpp-node/
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("infinity_pedal");
    publisher = node->create_publisher<std_msgs::msg::Bool>("infinity_pedal/triggered", 2);

    pedal_event_loop(context);
    rclcpp::shutdown();

    return result;
}

