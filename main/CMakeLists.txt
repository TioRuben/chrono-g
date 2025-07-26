idf_component_register(SRCS "main.c"
                                "components/cyan_stopwatch/cyan_stopwatch.c"
                                "components/yellow_stopwatch/yellow_stopwatch.c"
                                "components/artificial_horizon/artificial_horizon.c"
                    INCLUDE_DIRS "."
                                "components/cyan_stopwatch"
                                "components/yellow_stopwatch"
                                "components/artificial_horizon")