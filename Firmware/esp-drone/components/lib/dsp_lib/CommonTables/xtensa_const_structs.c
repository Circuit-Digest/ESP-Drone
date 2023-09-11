
#include "xtensa_const_structs.h"

/* Floating-point structs */
const xtensa_cfft_instance_f32 xtensa_cfft_sR_f32_len16 = {
	16, twiddleCoef_16, xtensaBitRevIndexTable16, XTENSABITREVINDEXTABLE_16_TABLE_LENGTH
};

const xtensa_cfft_instance_f32 xtensa_cfft_sR_f32_len32 = {
	32, twiddleCoef_32, xtensaBitRevIndexTable32, XTENSABITREVINDEXTABLE_32_TABLE_LENGTH
};

const xtensa_cfft_instance_f32 xtensa_cfft_sR_f32_len64 = {
	64, twiddleCoef_64, xtensaBitRevIndexTable64, XTENSABITREVINDEXTABLE_64_TABLE_LENGTH
};

const xtensa_cfft_instance_f32 xtensa_cfft_sR_f32_len128 = {
	128, twiddleCoef_128, xtensaBitRevIndexTable128, XTENSABITREVINDEXTABLE_128_TABLE_LENGTH
};

const xtensa_cfft_instance_f32 xtensa_cfft_sR_f32_len256 = {
	256, twiddleCoef_256, xtensaBitRevIndexTable256, XTENSABITREVINDEXTABLE_256_TABLE_LENGTH
};

const xtensa_cfft_instance_f32 xtensa_cfft_sR_f32_len512 = {
	512, twiddleCoef_512, xtensaBitRevIndexTable512, XTENSABITREVINDEXTABLE_512_TABLE_LENGTH
};

const xtensa_cfft_instance_f32 xtensa_cfft_sR_f32_len1024 = {
	1024, twiddleCoef_1024, xtensaBitRevIndexTable1024, XTENSABITREVINDEXTABLE_1024_TABLE_LENGTH
};

const xtensa_cfft_instance_f32 xtensa_cfft_sR_f32_len2048 = {
	2048, twiddleCoef_2048, xtensaBitRevIndexTable2048, XTENSABITREVINDEXTABLE_2048_TABLE_LENGTH
};

const xtensa_cfft_instance_f32 xtensa_cfft_sR_f32_len4096 = {
	4096, twiddleCoef_4096, xtensaBitRevIndexTable4096, XTENSABITREVINDEXTABLE_4096_TABLE_LENGTH
};



/* Structure for real-value inputs */
/* Floating-point structs */
const xtensa_rfft_fast_instance_f32 xtensa_rfft_fast_sR_f32_len32 = {
	{ 16, twiddleCoef_32, xtensaBitRevIndexTable32, XTENSABITREVINDEXTABLE_16_TABLE_LENGTH },
	32U,
	(float32_t *)twiddleCoef_rfft_32
};

const xtensa_rfft_fast_instance_f32 xtensa_rfft_fast_sR_f32_len64 = {
	 { 32, twiddleCoef_32, xtensaBitRevIndexTable32, XTENSABITREVINDEXTABLE_32_TABLE_LENGTH },
	64U,
	(float32_t *)twiddleCoef_rfft_64
};

const xtensa_rfft_fast_instance_f32 xtensa_rfft_fast_sR_f32_len128 = {
	{ 64, twiddleCoef_64, xtensaBitRevIndexTable64, XTENSABITREVINDEXTABLE_64_TABLE_LENGTH },
	128U,
	(float32_t *)twiddleCoef_rfft_128
};

const xtensa_rfft_fast_instance_f32 xtensa_rfft_fast_sR_f32_len256 = {
	{ 128, twiddleCoef_128, xtensaBitRevIndexTable128, XTENSABITREVINDEXTABLE_128_TABLE_LENGTH },
	256U,
	(float32_t *)twiddleCoef_rfft_256
};

const xtensa_rfft_fast_instance_f32 xtensa_rfft_fast_sR_f32_len512 = {
	{ 256, twiddleCoef_256, xtensaBitRevIndexTable256, XTENSABITREVINDEXTABLE_256_TABLE_LENGTH },
	512U,
	(float32_t *)twiddleCoef_rfft_512
};

const xtensa_rfft_fast_instance_f32 xtensa_rfft_fast_sR_f32_len1024 = {
	{ 512, twiddleCoef_512, xtensaBitRevIndexTable512, XTENSABITREVINDEXTABLE_512_TABLE_LENGTH },
	1024U,
	(float32_t *)twiddleCoef_rfft_1024
};

const xtensa_rfft_fast_instance_f32 xtensa_rfft_fast_sR_f32_len2048 = {
	{ 1024, twiddleCoef_1024, xtensaBitRevIndexTable1024, XTENSABITREVINDEXTABLE_1024_TABLE_LENGTH },
	2048U,
	(float32_t *)twiddleCoef_rfft_2048
};

const xtensa_rfft_fast_instance_f32 xtensa_rfft_fast_sR_f32_len4096 = {
	{ 2048, twiddleCoef_2048, xtensaBitRevIndexTable2048, XTENSABITREVINDEXTABLE_2048_TABLE_LENGTH },
	4096U,
	(float32_t *)twiddleCoef_rfft_4096
};
