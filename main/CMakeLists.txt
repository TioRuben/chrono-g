idf_component_register(SRCS "main.c"
                                "visibility_manager.c"
                                "components/cyan_stopwatch/cyan_stopwatch.c"
                                "components/yellow_stopwatch/yellow_stopwatch.c"
                                "components/imu/imu.c"
                    INCLUDE_DIRS "."
                                "components/cyan_stopwatch"
                                "components/yellow_stopwatch"
                                "components/imu")