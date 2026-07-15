#include <array>
#include <cstdint>
#include <new>
#include <cstddef>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#pragma once

template <typename T,size_t N>
class StaticPool{
    private:
        alignas(4) std::array<uint8_t,sizeof(T)*N> memory_block;
        std::array<bool,N> in_use;
        k_mutex pool_mutex;
    
    public:
        StaticPool(){
            in_use.fill(false);
            k_mutex_init(&pool_mutex);
        }
        
        void* allocate(){
            k_mutex_lock(&pool_mutex,K_FOREVER);
            void* ptr=nullptr;
            
            for (size_t i=0;i<N;i++){
                if(!in_use[i]){
                    in_use[i]=true;
                    ptr=&memory_block[i*sizeof(T)];
                    break;
                }    
            }
            k_mutex_unlock(&pool_mutex);
            
            if (ptr==nullptr){
                printk("[ERR] StaticPool Out of Memory! Pool Size: %zu\n", N);
            }
            return ptr;
        }
        
        void deallocate(void* ptr){
            if (ptr==nullptr){
                return;
            }
            
            k_mutex_lock(&pool_mutex,K_FOREVER);
            auto* byte_ptr=static_cast<uint8_t*>(ptr);
            auto* base_ptr=memory_block.data();
            
            if (byte_ptr>=base_ptr && byte_ptr<base_ptr+(sizeof(T)*N)){
                size_t index=(byte_ptr-base_ptr)/sizeof(T);
                in_use[index]=false;
            }
            else{
                printk("[ERR] Invalid pointer passed to deallocate\n");
            }
            k_mutex_unlock(&pool_mutex);
        }
    };
