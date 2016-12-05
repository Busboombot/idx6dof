/*
 *  Class for an IDX Robot Limit Switches
 */

#ifndef  idx_limit_h
#define idx_limit_h


class IDXLimit {

    public:

        IDXLimit();
        
        virtual void begin() = 0;
        
        virtual int limitValue() = 0;
        
        virtual bool isInLimit() = 0;
      
};

class IDXPinLimit :  public IDXLimit {

    public:

        IDXPinLimit(int pin);
      
        void begin();
        
        int limitValue();
        
        bool isInLimit();
    
    private:
        
        uint8_t pin;
        
        uint8_t p_value;
    
};

class IDXMcpLimit :  public IDXLimit {

    public:

        IDXMcpLimit(int pin, int mcp_address=0);
        
        void begin();
        
        int limitValue();
        
        bool isInLimit();
    
    private:
        
        uint8_t pin;
        
        uint8_t mcp_address;
        
        uint8_t p_value;
};


#endif