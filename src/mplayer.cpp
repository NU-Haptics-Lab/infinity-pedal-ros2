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
#include "udev.h"
#include <linux/hiddev.h>
#include <fcntl.h>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

bool STATE = false;

void event_callback(pedal_context *context, int event, void *client_data)
{
    std::cout <<"event: " << event << std::endl;
    
    switch(event)
    {
        case PEDAL_EVENT_UP_LEFT:
            STATE = false;
            break;
        case PEDAL_EVENT_UP_MIDDLE:
            STATE = false;
            break;
        case PEDAL_EVENT_UP_RIGHT:
            STATE = false;
            break;

        case PEDAL_EVENT_DOWN_LEFT:
            STATE = true;
            break;
        case PEDAL_EVENT_DOWN_MIDDLE:
            STATE = true;
            break;
        case PEDAL_EVENT_DOWN_RIGHT:
            STATE = true;
            break;

        default:
            STATE = false;
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
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("infinity_pedal");
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher = node->create_publisher<std_msgs::msg::Bool>("infinity_pedal/triggered", 2);


    char *devnode = NULL;
    int fd = -1;
    struct hiddev_event hid_event;
    unsigned short pedal_num = 0;

    result = pedal_find_devnode(&devnode);
    if (0 == result)
    {
        printf("pedal device node: %s\n", devnode);

        if ((fd = open(devnode, O_RDONLY)) < 0)
        {
            perror("failed to open pedal hid device node\n");
            result = 1;
            return result;
        }
    }

    // make lambda fcn for pedal_event_once
    auto pedalFcn = [&context](){
        pedal_event_loop(context);
    };

    // create a separate thread for the pedal event once
    std::thread pedalRead(pedalFcn);
    
    while (rclcpp::ok()){
        // ROS publishing
        auto msg = std_msgs::msg::Bool();
        msg.data = STATE;
        publisher->publish(msg);

        // use rclcpp to wait for 0.05 s (20 Hz, 50 ms)
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    rclcpp::shutdown();

    return result;
}

