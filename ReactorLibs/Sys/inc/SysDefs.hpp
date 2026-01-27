#pragma once

#define SINGLETON(x)                \
public:                             \
static x& GetInstance()             \
{                                   \
    static x instance;              \
    return instance;                \
}                                   \
private:                            \
x(const x&) = delete;               \
x& operator=(const x&) = delete;    \
x()


#define APPLICATION_OVERRIDE                    \
protected:                                      \
virtual void Start() override;               \
virtual void Update() override;              \
virtual const std::type_info& GetType() override {return typeid(*this);};

