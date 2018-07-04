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

#ifndef __FBT_CPU_H__
#define __FBT_CPU_H__
#include <mach/mtk_ppm_api.h>

struct fbt_jerk {
	int id;
	struct hrtimer timer;
	struct work_struct work;
};
struct fbt_proc {
	int active_jerk_id;
	struct fbt_jerk jerks[2];
};

struct fbt_frame_info {
	int target_fps;
	long long mips_diff;
	long long mips;
	unsigned long long frame_time;
	unsigned long long q2q_time;
	int count;
};

#define SEQ_printf(m, x...)\
do {\
	if (m)\
		seq_printf(m, x);\
	else\
		pr_debug(x);\
} while (0)

void fbt_cpu_exit(void);
int fbt_cpu_init(void);

extern unsigned long int min_boost_freq[3];
extern int fbt_reset_asfc(int level);
extern void fstb_game_mode_change(int is_game);

extern int set_cpuset(int cluster);
extern int set_idle_prefer(int enable);
extern int update_userlimit_cpu_freq(int kicker, int num_cluster,
		struct ppm_limit_data *freq_limit);
extern unsigned int mt_cpufreq_get_freq_by_idx(int id, int idx);
extern unsigned int mt_ppm_userlimit_freq_limit_by_others(
		unsigned int cluster);
extern void fbc_notify_game(int game);

void fbt_cpu_set_game_hint_cb(int is_game_mode);

#ifdef CONFIG_MTK_FPSGO_FBT_GAME
int switch_fbt_game(int);
#else
static inline int switch_fbt_game(int en) { return 0; }
#endif

#endif
