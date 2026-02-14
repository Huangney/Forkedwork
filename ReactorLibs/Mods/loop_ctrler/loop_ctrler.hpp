#pragma once

enum LoopOrder
{
    Pos,
    Speed,
};

class LoopCtrler
{
    public:
    LoopCtrler(){};

    virtual float Calc(float targ_value){return 0;};
    virtual void Observe(float pos, float spd, float u){};

    LoopOrder order = Speed;

};


