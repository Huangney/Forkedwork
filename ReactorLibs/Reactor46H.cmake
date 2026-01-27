# 这里的 ${PROJECT_SOURCE_DIR} 指的是CubeMX的那个 .ioc 文件所在的根目录
# 递归寻找所有 .h 和 .hpp 文件
file(GLOB_RECURSE ALL_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/*.h" 
    "${CMAKE_CURRENT_LIST_DIR}/*.hpp"
)

# 提取这些头文件所在的文件夹路径
set(REACTOR_INC "")
foreach(_file ${ALL_HEADERS})
    get_filename_component(_dir "${_file}" DIRECTORY)
    list(APPEND REACTOR_INC "${_dir}")
endforeach()

# 去重并清理
if(REACTOR_INC)
    list(REMOVE_DUPLICATES REACTOR_INC)
endif()

# 将这些路径注入到你的工程中
# 使用 BEFORE 关键字可以让你自己的库优先级高于 CubeMX 默认库
target_include_directories(${CMAKE_PROJECT_NAME} BEFORE PRIVATE ${REACTOR_INC})

# 收集所有 .c 源文件
file(GLOB_RECURSE REACTOR_SRCS 
    "ReactorLibs/*.c"
    "ReactorLibs/*.cpp"
)

# 将路径和文件添加到工程目标中
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${REACTOR_SRCS})

# 启用RTTI
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE 
    $<$<COMPILE_LANGUAGE:CXX>:-frtti>
)