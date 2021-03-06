#ifndef _X86_CURRENT_H
#define _X86_CURRENT_H

#if 1
extern struct task_struct *get_current(void);

#else // 1
#ifdef CONFIG_X86_32
#include <linux/compiler.h>
#include <asm/percpu.h>

struct task_struct;

DECLARE_PER_CPU(struct task_struct *, current_task);
static __always_inline struct task_struct *get_current(void)
{
	return x86_read_percpu(current_task);
}

#else /* X86_32 */

#ifndef __ASSEMBLY__
#include <asm/pda.h>

struct task_struct;

static __always_inline struct task_struct *get_current(void)
{
	return read_pda(pcurrent);
}

#else /* __ASSEMBLY__ */

#include <asm/asm-offsets.h>
#define GET_CURRENT(reg) movq %gs:(pda_pcurrent),reg

#endif /* __ASSEMBLY__ */

#endif /* X86_32 */
#endif // 1

#define current get_current()

#endif /* X86_CURRENT_H */
