#include <app/lib/blocks/glossy.h>

#include <zephyr/shell/shell.h>

#include <stdlib.h>

static int cmd_set_status_message(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_error(sh, "Usage: config status_message <0|1>");
        shell_help(sh);
        return -EINVAL;
    }

    bool value = (atoi(argv[1]) != 0);
    glossy_set_status_message(value);
    shell_print(sh, "Glossy status message set to: %d", value);

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(glossy_sub_config,
    SHELL_CMD(status_message, NULL, "Enable/disable glossy status messages", cmd_set_status_message),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(glossy, &glossy_sub_config, "Configure node local glossy settings", NULL);
