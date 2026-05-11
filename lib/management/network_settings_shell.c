/* SHELL_CMD_REGISTER(network, &network_sub_config, "Configure network settings", NULL); */

/* SHELL_STATIC_SUBCMD_SET_CREATE(sub_config, */
/*     SHELL_CMD(ranging_round, NULL, "Set ranging round phases and slots", cmd_set_ranging_round_phase_slots), */
/*     SHELL_CMD(commit_mode, NULL, "set how network settings are applied by a node", cmd_set_commit_mode), */
/*     SHELL_CMD(glossy_rangings, NULL, "setup number of rangings per glossy", cmd_set_glossy_rangings), */
/*     /\* SHELL_CMD(guards, NULL, "setup guard periods", cmd_set_guards), *\/ */
/*     SHELL_CMD(schedule_type, NULL, "Set schedule type (0=basic, 1=round_robin, 2=adaptive, 3=contention)", cmd_set_schedule_type), */
/*     SHELL_CMD(warmup_period, NULL, "Set initial warmup period in ms", cmd_set_warmup_period), */
/*     SHELL_CMD(slot_duration, NULL, "Set scheduler slot duration in ms", cmd_set_scheduler_slot_duration), */
/*     SHELL_CMD(status_message, NULL, "Enable/disable glossy status messages", cmd_set_status_message), */
/*     SHELL_CMD(show, NULL, "Show current configuration", cmd_show_config), */
/*     SHELL_SUBCMD_SET_END */
/* ); */

/* static int cmd_set_commit_mode(const struct shell *sh, size_t argc, char **argv) */
/* { */
/*     if (argc != 2) { */
/*         shell_error(sh, "Usage: config commit_mode <0|1>"); */
/*         shell_help(sh); */
/*         return -EINVAL; */
/*     } */

/*     bool value = (atoi(argv[1]) != 0); */
/*     network_config_commit_mode = value ? COMMIT_GLOSSY : COMMIT_DIRECT; */
/*     shell_print(sh, "Commit mode set to: %d", value); */

/*     // Save the setting */
/*     settings_save_one("ranging/commit_mode", &network_config_commit_mode, */
/*                     sizeof(network_config_commit_mode)); */

/*     return 0; */
/* } */
