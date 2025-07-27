idf_component_register(SRCS "main.c"
                                "components/cyan_stopwatch/cyan_stopwatch.c"
                                "components/yellow_stopwatch/yellow_stopwatch.c"
                                "components/artificial_horizon/artificial_horizon.c"
                                "components/artificial_horizon/artificial_horizon_ekf_wrapper.cpp"
                    INCLUDE_DIRS "."
                                "components/cyan_stopwatch"
                                "components/yellow_stopwatch"
                                "components/artificial_horizon")

# Enable C++ compilation for the ESP-DSP EKF wrapper
set_source_files_properties("components/artificial_horizon/artificial_horizon_ekf_wrapper.cpp" PROPERTIES COMPILE_FLAGS "-std=c++17")