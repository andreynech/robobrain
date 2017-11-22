#pragma once

#include <string>
#include <sstream>
#include <iostream>
#include "mosquittopp.h"


class RPCClient : public mosqpp::mosquittopp
{
public:
    RPCClient(const std::string &cli_id, 
              const std::string &svr_id,
              bool start_loop = false,
              bool clean_session = true) 
      : mosqpp::mosquittopp(cli_id.c_str(), clean_session),
        client_id(cli_id),
        server_id(svr_id),
        publish_topic(std::string("/") + svr_id + std::string("/") + cli_id + "/request"),
        subscribe_topic(std::string("/") + svr_id + std::string("/") + cli_id + "/response"),
        start_event_loop(start_loop)
    {
		int mid = 0;
		int res = connect("localhost", 1883, 60);
		if (res != 0)
		{
			std::cout << "Connect returned: " << res << " " << mosqpp::strerror(res) << std::endl;
			std::cout << mosqpp::connack_string(res) << std::endl;
		}
        res = subscribe(&mid, subscribe_topic.c_str());
		if (res)
			std::cout << "Subscribe returned: " << res << " " << mosqpp::strerror(res) << std::endl;
	
		if (start_event_loop)
		{
			threaded_set(true);
			loop_start();
		}
	}


    virtual ~RPCClient()
    {
        disconnect();
        if(start_event_loop)
            loop_stop();
    }


public:

    template <typename REQ_T, typename RESP_T>
    int call(const std::string &method_name, 
             const REQ_T &request,
             RESP_T &response)
    {
        std::ostringstream buffer;
        buffer << request;
        int mid = 0;
        int res = publish(&mid, 
                          publish_topic.c_str(),
                          buffer.str().size(),
                          buffer.str().c_str());
		if(res)
			std::cout << "Publish returned: " << res << " " << mosqpp::strerror(res) << std::endl;
        return res;
    }


    virtual void on_message(const struct mosquitto_message *message)
    {
       std::cout << "Message id: " << message->mid << std::endl;
       std::cout << "Topic: " << message->topic << std::endl;
       std::cout << "Payload len: " << message->payloadlen << std::endl;
       std::cout << "Payload: " << message->payload << std::endl;
    }

	virtual void on_error()
	{
		std::cout << "Error callback" << std::endl;
	}


private:
    const std::string client_id;
    const std::string server_id;
    const std::string publish_topic;
    const std::string subscribe_topic;
    bool start_event_loop;
};


