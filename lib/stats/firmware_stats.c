/*
 * On-demand firmware statistics collection -- core module.
 *
 * Static array of tracking slots with atomic value updates (ISR-safe).
 * Subscribe/unsubscribe protected by spinlock (BLE thread only).
 * Change-driven BLE notifications via k_work submission.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#include <app/lib/stats/firmware_stats.h>

LOG_MODULE_REGISTER(firmware_stats, CONFIG_LOG_DEFAULT_LEVEL);

/* ---- Slot storage ------------------------------------------------------- */

struct stat_slot {
	uint32_t stat_id;    /* 0 = unused */
	atomic_t value;
	atomic_t active;     /* 1 = in use */
	atomic_t dirty;      /* 1 = changed since last notify */
};

static struct stat_slot slots[CONFIG_FIRMWARE_STATS_MAX_SLOTS];
static struct k_spinlock slots_lock;  /* protects subscribe/unsubscribe only */

/* ---- Notification work -------------------------------------------------- */

static struct k_work notify_work;

/* Callback registered by the BLE service to send notifications.
 * Called from the system work queue thread context. */
static void (*notify_callback)(void);

static void notify_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (notify_callback) {
		notify_callback();
	}
}

void firmware_stats_set_notify_callback(void (*cb)(void))
{
	notify_callback = cb;
}

/* ---- Init (called automatically via SYS_INIT) --------------------------- */

static int firmware_stats_init(void)
{
	k_work_init(&notify_work, notify_work_handler);
	LOG_INF("Firmware stats initialized (%d slots)",
		CONFIG_FIRMWARE_STATS_MAX_SLOTS);
	return 0;
}

SYS_INIT(firmware_stats_init, APPLICATION, 90);

/* ---- Find slot by stat_id ----------------------------------------------- */

static struct stat_slot *find_slot(uint32_t stat_id)
{
	for (int i = 0; i < CONFIG_FIRMWARE_STATS_MAX_SLOTS; i++) {
		if (atomic_get(&slots[i].active) && slots[i].stat_id == stat_id) {
			return &slots[i];
		}
	}
	return NULL;
}

/* ---- Subscribe / Unsubscribe -------------------------------------------- */

int firmware_stats_subscribe(uint32_t stat_id)
{
	if (stat_id == 0) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&slots_lock);

	/* Check if already active */
	for (int i = 0; i < CONFIG_FIRMWARE_STATS_MAX_SLOTS; i++) {
		if (atomic_get(&slots[i].active) && slots[i].stat_id == stat_id) {
			k_spin_unlock(&slots_lock, key);
			return -EALREADY;
		}
	}

	/* Find a free slot */
	for (int i = 0; i < CONFIG_FIRMWARE_STATS_MAX_SLOTS; i++) {
		if (!atomic_get(&slots[i].active)) {
			slots[i].stat_id = stat_id;
			atomic_set(&slots[i].value, 0);
			atomic_set(&slots[i].dirty, 0);
			atomic_set(&slots[i].active, 1);
			k_spin_unlock(&slots_lock, key);
			LOG_INF("Subscribed to stat 0x%08x (slot %d)", stat_id, i);
			return 0;
		}
	}

	k_spin_unlock(&slots_lock, key);
	LOG_WRN("No free slots for stat 0x%08x", stat_id);
	return -ENOMEM;
}

int firmware_stats_unsubscribe(uint32_t stat_id)
{
	k_spinlock_key_t key = k_spin_lock(&slots_lock);

	for (int i = 0; i < CONFIG_FIRMWARE_STATS_MAX_SLOTS; i++) {
		if (atomic_get(&slots[i].active) && slots[i].stat_id == stat_id) {
			atomic_set(&slots[i].active, 0);
			slots[i].stat_id = 0;
			atomic_set(&slots[i].value, 0);
			atomic_set(&slots[i].dirty, 0);
			k_spin_unlock(&slots_lock, key);
			LOG_INF("Unsubscribed from stat 0x%08x (slot %d)", stat_id, i);
			return 0;
		}
	}

	k_spin_unlock(&slots_lock, key);
	return -ENOENT;
}

/* ---- Hot path: inc / add / set (ISR-safe) ------------------------------- */

void firmware_stats_inc(uint32_t stat_id)
{
	for (int i = 0; i < CONFIG_FIRMWARE_STATS_MAX_SLOTS; i++) {
		if (atomic_get(&slots[i].active) && slots[i].stat_id == stat_id) {
			atomic_inc(&slots[i].value);
			atomic_set(&slots[i].dirty, 1);
			k_work_submit(&notify_work);
			return;
		}
	}
}

void firmware_stats_add(uint32_t stat_id, uint32_t delta)
{
	for (int i = 0; i < CONFIG_FIRMWARE_STATS_MAX_SLOTS; i++) {
		if (atomic_get(&slots[i].active) && slots[i].stat_id == stat_id) {
			atomic_add(&slots[i].value, delta);
			atomic_set(&slots[i].dirty, 1);
			k_work_submit(&notify_work);
			return;
		}
	}
}

void firmware_stats_set(uint32_t stat_id, uint32_t value)
{
	for (int i = 0; i < CONFIG_FIRMWARE_STATS_MAX_SLOTS; i++) {
		if (atomic_get(&slots[i].active) && slots[i].stat_id == stat_id) {
			atomic_set(&slots[i].value, (atomic_val_t)value);
			atomic_set(&slots[i].dirty, 1);
			k_work_submit(&notify_work);
			return;
		}
	}
}

/* ---- Read / Reset / Query ----------------------------------------------- */

int firmware_stats_read(uint32_t stat_id, uint32_t *out_value)
{
	struct stat_slot *slot = find_slot(stat_id);
	if (!slot) {
		return -ENOENT;
	}
	*out_value = (uint32_t)atomic_get(&slot->value);
	return 0;
}

int firmware_stats_reset(uint32_t stat_id)
{
	struct stat_slot *slot = find_slot(stat_id);
	if (!slot) {
		return -ENOENT;
	}
	atomic_set(&slot->value, 0);
	return 0;
}

bool firmware_stats_is_active(uint32_t stat_id)
{
	return find_slot(stat_id) != NULL;
}

uint8_t firmware_stats_active_count(void)
{
	uint8_t count = 0;
	for (int i = 0; i < CONFIG_FIRMWARE_STATS_MAX_SLOTS; i++) {
		if (atomic_get(&slots[i].active)) {
			count++;
		}
	}
	return count;
}

/* ---- Dirty check (called by BLE service notify handler) ----------------- */

bool firmware_stats_check_and_clear_dirty(uint32_t stat_id, uint32_t *out_value)
{
	struct stat_slot *slot = find_slot(stat_id);
	if (!slot) {
		return false;
	}
	if (!atomic_cas(&slot->dirty, 1, 0)) {
		return false;
	}
	*out_value = (uint32_t)atomic_get(&slot->value);
	return true;
}
