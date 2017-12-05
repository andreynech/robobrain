#include <iostream>
#include <list>
#include "rpc_over_mqtt.h"
#include "process_nav_request.h"


#define CACHE_SIZE 100


class ParticleServer : public RPCServer
{
public:
    ParticleServer(const std::string &svr_id)
        : RPCServer(svr_id)
    {
    }


    virtual ~ParticleServer() {}

public:

    virtual void on_message(const struct mosquitto_message *message)
    {
        particle::LocRequest loc_request;
        particle::EstimatedLocation response;

        // Search for client_id string in topic which is in format
        // /server_id/client_id/request
        // Answer should be sent to the topic
        // /server_id/client_id/response
        const std::string topic(message->topic);
        size_t first_slash = topic.rfind("/");
        size_t second_slash = topic.rfind("/", first_slash - 1);
        const std::string client_id = topic.substr(second_slash, first_slash - second_slash);
        const std::string publish_topic = 
            std::string("/") + server_id + client_id 
            + std::string("/response");
        std::cout << "Publish topic: " << publish_topic << std::endl;

        request_cache_t::iterator search = cache.find(client_id);
        if(search == cache.end())
        {
            std::cout << "Cache miss" << std::endl;
            if(evictor_list.size() > CACHE_SIZE)            
            {
                // We need to remove LRU element
                request_cache_t::const_iterator lru = evictor_list.back();
                evictor_list.pop_back();
                cache.erase(lru);
            }
            auto res = cache.emplace(client_id, request_t());
            search = res.first;
        }
        else
        {
            std::cout << "Cache hit" << std::endl;
            // Move requested element to the head of evictor list (MRU)
            evictor_list.remove(search);
        }
        evictor_list.push_front(search);
        std::cout << "Evictor list size: " << evictor_list.size() << std::endl;

        bool ok = processLocRequest(loc_request, response, search->second,
                                    message->payload, message->payloadlen);

        std::string buffer;
        ok = response.SerializeToString(&buffer);
        if(!ok)
        {
            std::cout << "Response serialization error" << std::endl;
            return;
        }

        int mid = 0;
        int res = publish(&mid, 
                          publish_topic.c_str(),
                          buffer.size(),
                          buffer.c_str());
        if(res)
            std::cerr << "Publish returned: " << res << " " << mosqpp::strerror(res) << std::endl;
        else
            std::cerr << "Publish message id: " << mid << std::endl;
    }

protected:
    // Evictor pattern implementation for cache
    typedef std::map<std::string, request_t> request_cache_t;
    typedef std::list<request_cache_t::const_iterator> evictor_list_t;
	request_cache_t cache;
    evictor_list_t evictor_list;
};


int main(int argc, char *argv[])
{
    // Verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    GOOGLE_PROTOBUF_VERIFY_VERSION;

	mosquitto_lib_init();

	ParticleServer server("server1");
    server.run();

	return 0;
}

