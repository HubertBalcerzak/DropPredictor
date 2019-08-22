#include <iostream>
#include <cmath>
#include <memory>


class Vector3D{
public:
    Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}

    double x;
    double y;
    double z;

    Vector3D operator + (const Vector3D & other) const{
        return Vector3D {x + other.x, y + other.y, z + other.z};
    }

    Vector3D& operator +=(const Vector3D& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3D operator - (const Vector3D & other) const{
        return Vector3D {x - other.x, y - other.y, z - other.z};
    }

    Vector3D operator - () const{
        return Vector3D {-x, -y, -z};
    }

    template <typename T>
    Vector3D operator * (const T& scalar) const{
        return Vector3D {scalar * x, scalar * y, scalar * z};
    }
    template <typename T>
    Vector3D operator / (const T& scalar) const{
        return Vector3D {x/scalar, y/scalar, z/scalar};
    }

    Vector3D cross(const Vector3D &other) const{
        return Vector3D{y * other.z - z * other.y, x * other.z - z * other.x, x * other.y - y * other.x};
    }

    double dot(const Vector3D &other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    double len() const{
        return std::sqrt(x * x + y * y + z * z);
    }
    Vector3D normalized() const {
        return *(this)/len();
    }
};

const Vector3D gravity = Vector3D{0, -9.81, 0};


Vector3D calculateDrag(double Cd, double Area, Vector3D velocity){
    double airDensity = 1.2;
    return -velocity.normalized() //zwrot przeciwny do prędkości
           * Cd * Area * airDensity * velocity.len() * velocity.len() / 2; //wartość
}

std::ostream& operator << (std::ostream& stream, const Vector3D &vector3D){
    return stream<<"["<<vector3D.x<<", "<<vector3D.y<<", "<<vector3D.z<<"]\n";
}


class State{
public:
    Vector3D positon;
    Vector3D velocity;
    double mass;
    double area;

    State(const Vector3D &positon, const Vector3D &velocity, double mass, double area) :
            positon(positon),
            velocity(velocity), mass(mass),
            area(area) {}
};

class RHSFunction{
public:
    State operator()(const State &oldState, double dt) {
        Vector3D acceleration = (calculateDrag(0.9, oldState.area, oldState.velocity) + gravity)/oldState.mass;
        Vector3D velocity =oldState.velocity + acceleration * dt;
        Vector3D position =oldState.positon + velocity * dt;
        return State{position, velocity, oldState.mass, oldState.area};
    }

};

class EndCondition{
public:
    bool isFinished(const State& state){
        return state.positon.y <= 0;
    }
};

class EulerMethod{
public:
    State state;
    RHSFunction rhsFunction;
    EulerMethod(const State &state, const RHSFunction &rhsFunction) : state(state), rhsFunction(rhsFunction) {}

    State calculateStateAtEndCondition(EndCondition endCondition){
        double step = 0.001;
        while (!endCondition.isFinished(state)){
            advanceByDt(step);
        }
        return state;
    }

    void advanceByDt(double dt){
        state = rhsFunction(state, dt);
    }

};




int main() {
    State initialState{
        Vector3D{0, 10, 0},
        Vector3D{10, 0, 0},
        0.1,
        3.13 * 0.05 * 0.05
    };
    EulerMethod eulerMethod{
        initialState,
        RHSFunction{},
    };
    State stanKoncowy = eulerMethod.calculateStateAtEndCondition(EndCondition{});
    std::cout << stanKoncowy.positon << stanKoncowy.velocity;
    return 0;
}