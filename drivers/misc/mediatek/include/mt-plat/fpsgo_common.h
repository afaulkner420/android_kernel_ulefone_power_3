/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _FPSGO_COMMON_H_
#define _FPSGO_COMMON_H_

#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/fs.h>

struct fps_level {
	int start;
	int end;
};

#ifdef CONFIG_MTK_FPSGO
#define FPSGO_SYSTRACE_LIST(macro) \
	macro(MANDATORY, 0), \
	macro(FBT_GM, 1), \
	macro(FBT_UX, 2), \
	macro(FSTB, 3), \
	macro(XGF, 4), \
	macro(NTFR, 5), \
	macro(MAX, 6), \

#define GENERATE_ENUM(name, shft) FPSGO_DEBUG_##name = 1U << shft
enum {
	FPSGO_SYSTRACE_LIST(GENERATE_ENUM)
};

extern uint32_t fpsgo_systrace_mask;
extern struct dentry *fpsgo_debugfs_dir;
extern int game_ppid;

void __fpsgo_systrace_c(pid_t, int, const char *, ...);
void __fpsgo_systrace_b(pid_t, const char *, ...);
void __fpsgo_systrace_e(void);

#define fpsgo_systrace_c(mask, pid, val, fmt...) \
	do { \
		if (fpsgo_systrace_mask & mask) \
			__fpsgo_systrace_c(pid, val, fmt); \
	} while (0)

#define fpsgo_systrace_b(mask, tgid, fmt, ...) \
	do { \
		if (fpsgo_systrace_mask & mask) \
			__fpsgo_systrace_b(tgid, fmt); \
	} while (0)

#define fpsgo_systrace_e(mask) \
	do { \
		if (fpsgo_systrace_mask & mask) \
			__fpsgo_systrace_e(); \
	} while (0)

#define fpsgo_systrace_c_fbt_gm(pid, val, fmt...) \
	fpsgo_systrace_c(FPSGO_DEBUG_FBT_GM, pid, val, fmt)
#define fpsgo_systrace_c_fbt_ux(pid, val, fmt...) \
	fpsgo_systrace_c(FPSGO_DEBUG_FBT_UX, pid, val, fmt)
#define fpsgo_systrace_c_fstb(pid, val, fmt...) \
	fpsgo_systrace_c(FPSGO_DEBUG_FSTB, pid, val, fmt)
#define fpsgo_systrace_c_xgf(pid, val, fmt...) \
	fpsgo_systrace_c(FPSGO_DEBUG_XGF, pid, val, fmt)
#define fpsgo_systrace_c_ntfr(pid, val, fmt...) \
	fpsgo_systrace_c(FPSGO_DEBUG_NTFR, pid, val, fmt)

#define fpsgo_systrace_c_log(val, fmt...) \
	do { \
		if (game_ppid > 0) \
			fpsgo_systrace_c(FPSGO_DEBUG_MANDATORY, game_ppid, val, fmt); \
	} while (0)

int fpsgo_is_fstb_enable(void);
int fpsgo_switch_fstb(int);
int fpsgo_fstb_sample_window(long long time_usec);
int fpsgo_fstb_fps_range(int nr_level, struct fps_level *level);
int fpsgo_fstb_fps_error_threhosld(int threshold);
int fpsgo_fstb_percentile_frametime(int ratio);

int fpsgo_switch_fbt_game(int);

int fbt_cpu_set_bhr(int new_bhr);
int fbt_cpu_set_bhr_opp(int new_opp);
int fbt_cpu_set_rescue_opp_c(int new_opp);
int fbt_cpu_set_rescue_opp_f(int new_opp);
int fbt_cpu_set_rescue_percent(int percent);
int fbt_cpu_set_variance(int var);
int fbt_cpu_set_floor_bound(int bound);
int fbt_cpu_set_floor_kmin(int k);
int fbt_cpu_set_floor_opp(int new_opp);

#else
static inline void fpsgo_systrace_c(uint32_t m, pid_t id, int val,
				    const char *s, ...) { }
static inline void fpsgo_systrace_b(uint32_t m, pid_t id,
				    const char *s, ...) { }
static inline void fpsgo_systrace_e(uint32_t m) { }


static inline void fpsgo_systrace_c_fbt_gm(pid_t id, int val,
					const char *s, ...) { }
static inline void fpsgo_systrace_c_fbt_ux(pid_t id, int val,
					const char *s, ...) { }
static inline void fpsgo_systrace_c_fstb(pid_t id, int val,
					const char *s, ...) { }
static inline void fpsgo_systrace_c_xgf(pid_t id, int val,
					const char *s, ...) { }
static inline void fpsgo_systrace_c_ntfr(pid_t id, int val,
					const char *s, ...) { }
static inline void fpsgo_systrace_c_log(pid_t id, int val,
					const char *s, ...) { }

static inline int fpsgo_is_fstb_enable(void) { return 0; }
static inline int fpsgo_switch_fstb(int en) { return 0; }
static inline int fpsgo_fstb_sample_window(long long time_usec) { return 0; }
static inline int fpsgo_fstb_fps_range(int nr_level, struct fps_level *level) { return 0; }
static inline int fpsgo_fstb_fps_error_threhosld(int threshold) { return 0; }
static inline int fpsgo_fstb_percentile_frametime(int ratio) { return 0; }

static inline int fpsgo_switch_fbt_game(int en) { return 0; }

static inline int fbt_cpu_set_bhr(int new_bhr) { return 0; }
static inline int fbt_cpu_set_bhr_opp(int new_opp) { return 0; }
static inline int fbt_cpu_set_rescue_opp_c(int new_opp) { return 0; }
static inline int fbt_cpu_set_rescue_opp_f(int new_opp) { return 0; }
static inline int fbt_cpu_set_rescue_percent(int percent) { return 0; }
static inline int fbt_cpu_set_variance(int var) { return 0; }
static inline int fbt_cpu_set_floor_bound(int bound) { return 0; }
static inline int fbt_cpu_set_floor_kmin(int k) { return 0; }
static inline int fbt_cpu_set_floor_opp(int new_opp) { return 0; }
#endif

#ifdef CONFIG_MTK_FPSGO_FBT_GAME
void xgf_igather_timer(const struct hrtimer * const, int);
#else
static inline void xgf_igather_timer(const struct hrtimer * const t, int v) { }
#endif

#endif
