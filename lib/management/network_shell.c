#include <app/lib/node_table/node_table.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

#define MAX_NODES CONFIG_NODE_TABLE_MAX_ENTRIES

static int cmd_set_system_addresses(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: config addrs <id1> <id2> ...");
        shell_help(sh);
        return -EINVAL;
    }

    size_t count = argc - 1;
    if (count > MAX_NODES) {
        shell_error(sh, "Too many addresses. Maximum is %d", MAX_NODES);
        return -EINVAL;
    }

    /* Clear the existing node table */
    node_table_clear();

    /* Parse and add each node to the table */
    for (size_t i = 0; i < count; i++) {
        int id = atoi(argv[i + 1]);
        if (id < 0 || id > 0xFFFF) {
            shell_error(sh, "Invalid node ID: %d. Must be between 0 and 65535", id);
            return -EINVAL;
        }
        /* Add node with dummy distance to create entry */
        node_table_update((uint16_t)id, 0, 0);
    }
    node_table_notify_changed();

    shell_print(sh, "Node table updated with %zu nodes", node_table_get_count());

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_config,
    SHELL_CMD(addrs, NULL, "setup known addrs of system", cmd_set_system_addresses));
SHELL_CMD_REGISTER(config, &sub_config, "Configure ranging settings", NULL);
