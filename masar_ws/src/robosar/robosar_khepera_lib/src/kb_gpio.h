/*-------------------------------------------------------------------------------
 * Project: KoreBot Library	
 * $Author: flambercy $
 * $Date: 2006/01/26 14:50:58 $
 * $Revision: 1.2 $
 * 
 * 
 * $Header: /home/cvs/libkhepera/src/kb_gpio.h,v 1.2 2006/01/26 14:50:58 flambercy Exp $
 */
  
#ifndef __gpio__
#define __gpio__


#define GPIO_OUTPUT 0
#define GPIO_INPUT  1

#define GPIO_FIRST 70
#define GPIO_LAST	87

#define GPIO_SMUTE 64

#define GPIO_KH4_RESET 90


extern int kb_gpio_init();
extern int kb_gpio_cleanup();

extern int kb_gpio_dir(unsigned gpio, unsigned dir);
extern int kb_gpio_dir_val(unsigned gpio, unsigned dir, unsigned value);

extern void kb_gpio_set(unsigned gpio);
extern void kb_gpio_clear(unsigned gpio);
extern int kb_gpio_get(unsigned gpio);

void kb_gpio_function(unsigned gpio, unsigned function);

#endif /* __gpio__ */
