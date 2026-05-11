#include <zephyr/shell/shell.h>
#include <zephyr/settings/settings.h>
#include <app/lib/system/node.h>
#include <stdlib.h>

static int cmd_set_node_mode(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_error(sh, "Usage: config node_mode <mode>");
        shell_help(sh);
        return -EINVAL;
    }

    int mode = atoi(argv[1]);
    if (mode < NODE_MODE_DISABLED || mode > NODE_MODE_RANGING_DISABLED) {
        shell_error(sh, "Invalid mode. Use 0 (disabled), 1 (passive), 2 (active) or 3 (ranging disabled)");
        return -EINVAL;
    }

    set_node_mode((node_mode_t)mode);
    shell_print(sh, "Node mode set to: %d", mode);

    // Save the setting
    node_mode_t node_mode = (node_mode_t)mode;
    settings_save_one("node/node_mode", &node_mode, sizeof(node_mode));

    return 0;
}

/* Shell commands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_config,
    SHELL_CMD(node_mode, NULL, "Set node mode (0=disabled, 1=passive, 2=active)", cmd_set_node_mode)
    /* SHELL_CMD(node_id, NULL, "Set node ranging ID", cmd_set_node_id) */
    );
SHELL_CMD_REGISTER(config, &sub_config, "Configure ranging settings", NULL);