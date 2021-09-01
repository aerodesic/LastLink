/*
 * power management via axp192 driver
 */

#ifndef __power_manager_h_included
#define __power_manager_h_included

void start_axp192(void);

bool axp192_enable_ldo2(bool on);
int axp192_test_ldo2(void);

bool axp192_enable_ldo3(bool on);
int axp192_test_ldo3(void);

bool axp192_enable_dcdc1(bool on);
int axp192_test_dcdc1(void);

bool axp192_enable_dcdc3(bool on);
int axp192_test_dcdc3(void);

#endif /* __power_manager_h_included */
