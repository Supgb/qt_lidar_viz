#ifndef LIDARDEBUG_H
#define LIDARDEBUG_H

namespace boost
{
class thread;
}

namespace lidar_base
{

template <typename T>
class LidarDebug final
{
public:
    explicit LidarDebug(T* t): _dbg_obj(t){}
    //LidarDebug();
    ~LidarDebug() = default;

    void debug();
    boost::thread* spawn();

private:
    T* _dbg_obj;
};

}   // namespace lidar_base

#endif // LIDARDEBUG_H
