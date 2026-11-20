cmake_minimum_required(VERSION 3.8)
project(bajie_sim)

find_package(ament_cmake REQUIRED)

# 把三维资源放进 share/bajie_sim/，保持 scene / machine / models 的相对结构，
# 否则 USD 内部的相对引用（../machine/... 、../models/...）会全部失效。
install(DIRECTORY scene machine models
        DESTINATION share/${PROJECT_NAME})

ament_package()
