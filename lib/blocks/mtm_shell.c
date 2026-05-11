#include <app/lib/blocks/mtm.h>

#include <zephyr/shell/shell.h>

#include <stdlib.h>

static int cmd_set_reject_frames(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_error(sh, "Usage: config reject_frames <0|1>");
        shell_help(sh);
        return -EINVAL;
    }

    bool value = (atoi(argv[1]) != 0);
    mtm_set_correct_reject_frames(value);
    shell_print(sh, "Frame rejection set to: %d", value);

    return 0;
}

static int cmd_set_fp_threshold(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_error(sh, "Usage: config fp_threshold <value>");
        shell_help(sh);
        return -EINVAL;
    }

    uint16_t value = atoi(argv[1]);
    mtm_set_fp_index_threshold(value);
    shell_print(sh, "FP index threshold set to: %u", value);

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(mtm_sub_config,
    SHELL_CMD(reject_frames, NULL, "Enable/disable frame rejection", cmd_set_reject_frames),
    SHELL_CMD(fp_threshold, NULL, "Set FP index threshold", cmd_set_fp_threshold),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(mtm, &mtm_sub_config, "Configure mtm node local settings", NULL);
