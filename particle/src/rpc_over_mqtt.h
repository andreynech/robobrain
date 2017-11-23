#pragma once

#include <future>
#include <string>
#include <sstream>
#include <iostream>
#include "mosquittopp.h"


template <typename REQ_T, typename RESP_T>
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
            std::cerr << "Connect returned: " << res << " " << mosqpp::strerror(res) << std::endl;
            std::cerr << mosqpp::connack_string(res) << std::endl;
        }
        res = subscribe(&mid, subscribe_topic.c_str());
        if (res)
            std::cerr << "Subscribe returned: " << res << " " << mosqpp::strerror(res) << std::endl;
	
		if (start_event_loop)
		{
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
        {
            std::cerr << "Publish returned: " << res << " " << mosqpp::strerror(res) << std::endl;
        }
        else
        {
            std::cerr << "Publish message id: " << mid << std::endl;
            std::future<RESP_T> response_future = this->response_promise.get_future();
            response_future.wait();
            response = response_future.get();
            // "Reinitialize" promise for the next call
            response_promise = response_promise_t();
		}

        return res;
    }


    virtual void on_message(const struct mosquitto_message *message)
    {
        std::cerr << "Message id: " << message->mid << std::endl;
        std::cerr << "Topic: " << message->topic << std::endl;
        std::cerr << "Payload len: " << message->payloadlen << std::endl;
        std::cerr << "Payload: " << message->payload << std::endl;

        this->response_promise.set_value(RESP_T((const char*)(message->payload)));
    }

	virtual void on_error()
	{
        std::cerr << "Error callback" << std::endl;
	}


private:
    const std::string client_id;
    const std::string server_id;
    const std::string publish_topic;
    const std::string subscribe_topic;
    bool start_event_loop;

    typedef std::promise<RESP_T> response_promise_t;
    response_promise_t response_promise;
};


template <typename REQ_T, typename RESP_T>
class RPCServer : public mosqpp::mosquittopp
{
public:
    RPCServer(const std::string &svr_id,
              bool start_loop = false,
              bool clean_session = true) 
      : mosqpp::mosquittopp(svr_id.c_str(), clean_session),
        server_id(svr_id),
        subscribe_topic(std::string("/") + svr_id + std::string("/+/request")),
        start_event_loop(start_loop)
    {
        int mid = 0;
        int res = connect("localhost", 1883, 60);
        if (res != 0)
        {
            std::cerr << "Connect returned: " << res << " " << mosqpp::strerror(res) << std::endl;
            std::cerr << mosqpp::connack_string(res) << std::endl;
        }
        res = subscribe(&mid, subscribe_topic.c_str());
        if (res)
            std::cerr << "Subscribe returned: " << res << " " << mosqpp::strerror(res) << std::endl;
	
	}


    virtual ~RPCServer()
    {
        disconnect();
        if(start_event_loop)
            loop_stop();
    }


public:

    void run()
    {
		if (start_event_loop)
		{
			loop_start();
		}
        else
        {
            loop_forever();
        }
    }


    virtual void on_message(const struct mosquitto_message *message)
    {
        std::cerr << "Message id: " << message->mid << std::endl;
        std::cerr << "Topic: " << message->topic << std::endl;
        std::cerr << "Payload len: " << message->payloadlen << std::endl;
        std::cerr << "Payload: " << (const char*)(message->payload) << std::endl;
    }


	virtual void on_error()
	{
        std::cerr << "Error callback" << std::endl;
	}


protected:
    const std::string server_id;
    const std::string subscribe_topic;
    bool start_event_loop;
};

