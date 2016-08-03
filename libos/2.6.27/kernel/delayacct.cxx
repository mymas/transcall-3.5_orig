



extern "C" {


extern void g_pause(void);
extern void g_unpause(void);
extern void g_init(int domain_id);
extern void g_exit(void);
extern void *g_map(void *addr, unsigned long size);
extern void g_unmap(void *laddr);
extern void *g_proc_map(void *addr, unsigned long size, void *pgd);





}




extern int pid_max;
extern int ngroups_max;



struct seq_file;
struct dentry_operations;
struct dentry;
struct inode_operations;
struct nameidata;
struct super_operations;
struct kstatfs;
struct vfsmount;
struct statfs;

struct seq_file *seq_get(char *data, int size);
struct seq_file *seq_renew(struct seq_file *m);

char *dentry_op_dname(struct dentry_operations *op,
                      struct dentry *dentry, char *buffer, int buflen);
void *inode_op_follow_link(struct inode_operations *op,
                           struct dentry *dentry, struct nameidata *nd);
int super_op_statfs(struct super_operations *op,
                    struct dentry *dentry, struct kstatfs *buf);


struct vfsmount *lookup_vfsmount(struct vfsmount *mnt, struct dentry *dentry);
int get_stat(struct vfsmount *mnt, struct dentry *dentry,
             struct kstat *stat);
int get_statfs(struct vfsmount *mnt, struct dentry *dentry,
               struct statfs *buf);




int uptime_proc_show(struct seq_file *m, void *v);
int meminfo_proc_show(struct seq_file *m, void *v);
int filesystems_proc_show(struct seq_file *m, void *v);
int version_proc_show(struct seq_file *m, void *v);
struct sched_param {
 int sched_priority;
};













enum {
 _false = 0,
 _true = 1
};
typedef struct {
 unsigned long fds_bits [(1024/(8 * sizeof(unsigned long)))];
} __kernel_fd_set;


typedef void (*__kernel_sighandler_t)(int);


typedef int __kernel_key_t;
typedef int __kernel_mqd_t;





typedef unsigned long __kernel_ino_t;
typedef unsigned int __kernel_mode_t;
typedef unsigned long __kernel_nlink_t;
typedef long __kernel_off_t;
typedef int __kernel_pid_t;
typedef int __kernel_ipc_pid_t;
typedef unsigned int __kernel_uid_t;
typedef unsigned int __kernel_gid_t;
typedef unsigned long __kernel_size_t;
typedef long __kernel_ssize_t;
typedef long __kernel_ptrdiff_t;
typedef long __kernel_time_t;
typedef long __kernel_suseconds_t;
typedef long __kernel_clock_t;
typedef int __kernel_timer_t;
typedef int __kernel_clockid_t;
typedef int __kernel_daddr_t;
typedef char * __kernel_caddr_t;
typedef unsigned short __kernel_uid16_t;
typedef unsigned short __kernel_gid16_t;


typedef long long __kernel_loff_t;


typedef struct {
 int val[2];
} __kernel_fsid_t;

typedef unsigned short __kernel_old_uid_t;
typedef unsigned short __kernel_old_gid_t;
typedef __kernel_uid_t __kernel_uid32_t;
typedef __kernel_gid_t __kernel_gid32_t;

typedef unsigned long __kernel_old_dev_t;




static inline void __FD_SET(unsigned long fd, __kernel_fd_set *fdsetp)
{
 unsigned long _tmp = fd / (8 * sizeof(unsigned long));
 unsigned long _rem = fd % (8 * sizeof(unsigned long));
 fdsetp->fds_bits[_tmp] |= (1UL<<_rem);
}


static inline void __FD_CLR(unsigned long fd, __kernel_fd_set *fdsetp)
{
 unsigned long _tmp = fd / (8 * sizeof(unsigned long));
 unsigned long _rem = fd % (8 * sizeof(unsigned long));
 fdsetp->fds_bits[_tmp] &= ~(1UL<<_rem);
}


static inline int __FD_ISSET(unsigned long fd, __const__ __kernel_fd_set *p)
{
 unsigned long _tmp = fd / (8 * sizeof(unsigned long));
 unsigned long _rem = fd % (8 * sizeof(unsigned long));
 return (p->fds_bits[_tmp] & (1UL<<_rem)) != 0;
}






static inline void __FD_ZERO(__kernel_fd_set *p)
{
 unsigned long *tmp = p->fds_bits;
 int i;

 if (__builtin_constant_p((1024/(8 * sizeof(unsigned long))))) {
  switch ((1024/(8 * sizeof(unsigned long)))) {
  case 32:
   tmp[ 0] = 0; tmp[ 1] = 0; tmp[ 2] = 0; tmp[ 3] = 0;
   tmp[ 4] = 0; tmp[ 5] = 0; tmp[ 6] = 0; tmp[ 7] = 0;
   tmp[ 8] = 0; tmp[ 9] = 0; tmp[10] = 0; tmp[11] = 0;
   tmp[12] = 0; tmp[13] = 0; tmp[14] = 0; tmp[15] = 0;
   tmp[16] = 0; tmp[17] = 0; tmp[18] = 0; tmp[19] = 0;
   tmp[20] = 0; tmp[21] = 0; tmp[22] = 0; tmp[23] = 0;
   tmp[24] = 0; tmp[25] = 0; tmp[26] = 0; tmp[27] = 0;
   tmp[28] = 0; tmp[29] = 0; tmp[30] = 0; tmp[31] = 0;
   return;
  case 16:
   tmp[ 0] = 0; tmp[ 1] = 0; tmp[ 2] = 0; tmp[ 3] = 0;
   tmp[ 4] = 0; tmp[ 5] = 0; tmp[ 6] = 0; tmp[ 7] = 0;
   tmp[ 8] = 0; tmp[ 9] = 0; tmp[10] = 0; tmp[11] = 0;
   tmp[12] = 0; tmp[13] = 0; tmp[14] = 0; tmp[15] = 0;
   return;
  case 8:
   tmp[ 0] = 0; tmp[ 1] = 0; tmp[ 2] = 0; tmp[ 3] = 0;
   tmp[ 4] = 0; tmp[ 5] = 0; tmp[ 6] = 0; tmp[ 7] = 0;
   return;
  case 4:
   tmp[ 0] = 0; tmp[ 1] = 0; tmp[ 2] = 0; tmp[ 3] = 0;
   return;
  }
 }
 i = (1024/(8 * sizeof(unsigned long)));
 while (i) {
  i--;
  *tmp = 0;
  tmp++;
 }
}



typedef __signed__ char __s8;
typedef unsigned char __u8;

typedef __signed__ short __s16;
typedef unsigned short __u16;

typedef __signed__ int __s32;
typedef unsigned int __u32;


__extension__ typedef __signed__ long long __s64;
__extension__ typedef unsigned long long __u64;
typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;



typedef unsigned short umode_t;
typedef u64 dma64_addr_t;


typedef u64 dma_addr_t;



typedef __u32 __kernel_dev_t;

typedef __kernel_fd_set fd_set;
typedef __kernel_dev_t dev_t;
typedef __kernel_ino_t ino_t;
typedef __kernel_mode_t mode_t;
typedef __kernel_nlink_t nlink_t;
typedef __kernel_off_t off_t;
typedef __kernel_pid_t pid_t;
typedef __kernel_daddr_t daddr_t;
typedef __kernel_key_t key_t;
typedef __kernel_suseconds_t suseconds_t;
typedef __kernel_timer_t timer_t;
typedef __kernel_clockid_t clockid_t;
typedef __kernel_mqd_t mqd_t;






typedef __kernel_uid32_t uid_t;
typedef __kernel_gid32_t gid_t;
typedef __kernel_uid16_t uid16_t;
typedef __kernel_gid16_t gid16_t;

typedef unsigned long uintptr_t;



typedef __kernel_old_uid_t old_uid_t;
typedef __kernel_old_gid_t old_gid_t;
typedef __kernel_loff_t loff_t;
typedef __kernel_size_t size_t;




typedef __kernel_ssize_t ssize_t;




typedef __kernel_ptrdiff_t ptrdiff_t;




typedef __kernel_time_t time_t;




typedef __kernel_clock_t clock_t;




typedef __kernel_caddr_t caddr_t;



typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;


typedef unsigned char unchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;




typedef __u8 u_int8_t;
typedef __s8 int8_t;
typedef __u16 u_int16_t;
typedef __s16 int16_t;
typedef __u32 u_int32_t;
typedef __s32 int32_t;



typedef __u8 uint8_t;
typedef __u16 uint16_t;
typedef __u32 uint32_t;


typedef __u64 uint64_t;
typedef __u64 u_int64_t;
typedef __s64 int64_t;
typedef unsigned long sector_t;
typedef unsigned long blkcnt_t;
typedef __u16 __le16;
typedef __u16 __be16;
typedef __u32 __le32;
typedef __u32 __be32;

typedef __u64 __le64;
typedef __u64 __be64;

typedef __u16 __sum16;
typedef __u32 __wsum;


typedef unsigned gfp_t;


typedef u64 resource_size_t;




struct ustat {
 __kernel_daddr_t f_tfree;
 __kernel_ino_t f_tinode;
 char f_fname[6];
 char f_fpack[6];
};

struct task_struct;
typedef struct __user_cap_header_struct {
 __u32 version;
 int pid;
} *cap_user_header_t;

typedef struct __user_cap_data_struct {
        __u32 effective;
        __u32 permitted;
        __u32 inheritable;
} *cap_user_data_t;
struct vfs_cap_data {
 __le32 magic_etc;
 struct {
  __le32 permitted;
  __le32 inheritable;
 } data[2];
};
typedef struct kernel_cap_struct {
 __u32 cap[2];
} kernel_cap_t;
static inline kernel_cap_t cap_combine( kernel_cap_t a,
           kernel_cap_t b)
{
 kernel_cap_t dest;
 do { unsigned __capi; for (__capi = 0; __capi < 2; ++__capi) { dest.cap[__capi] = a.cap[__capi] | b.cap[__capi]; } } while (0);
 return dest;
}

static inline kernel_cap_t cap_intersect( kernel_cap_t a,
      kernel_cap_t b)
{
 kernel_cap_t dest;
 do { unsigned __capi; for (__capi = 0; __capi < 2; ++__capi) { dest.cap[__capi] = a.cap[__capi] & b.cap[__capi]; } } while (0);
 return dest;
}

static inline kernel_cap_t cap_drop( kernel_cap_t a,
        kernel_cap_t drop)
{
 kernel_cap_t dest;
 do { unsigned __capi; for (__capi = 0; __capi < 2; ++__capi) { dest.cap[__capi] = a.cap[__capi] &~ drop.cap[__capi]; } } while (0);
 return dest;
}

static inline kernel_cap_t cap_invert( kernel_cap_t c)
{
 kernel_cap_t dest;
 do { unsigned __capi; for (__capi = 0; __capi < 2; ++__capi) { dest.cap[__capi] = ~ c.cap[__capi]; } } while (0);
 return dest;
}

static inline int cap_isclear( kernel_cap_t a)
{
 unsigned __capi;
 for (__capi = 0; __capi < 2; ++__capi) {
  if (a.cap[__capi] != 0)
   return 0;
 }
 return 1;
}

static inline int cap_issubset( kernel_cap_t a, kernel_cap_t set)
{
 kernel_cap_t dest;
 dest = cap_drop(a, set);
 return cap_isclear(dest);
}



static inline int cap_is_fs_cap(int cap)
{
 kernel_cap_t __cap_fs_set = ((kernel_cap_t){{ ((1 << ((0) & 31)) | (1 << ((27) & 31)) | (1 << ((1) & 31)) | (1 << ((2) & 31)) | (1 << ((3) & 31)) | (1 << ((4) & 31))) | (1 << ((9) & 31)), ((1 << ((32) & 31))) } });
 return !!((1 << ((cap) & 31)) & __cap_fs_set.cap[((cap) >> 5)]);
}

static inline kernel_cap_t cap_drop_fs_set( kernel_cap_t a)
{
 kernel_cap_t __cap_fs_set = ((kernel_cap_t){{ ((1 << ((0) & 31)) | (1 << ((27) & 31)) | (1 << ((1) & 31)) | (1 << ((2) & 31)) | (1 << ((3) & 31)) | (1 << ((4) & 31))) | (1 << ((9) & 31)), ((1 << ((32) & 31))) } });
 return cap_drop(a, __cap_fs_set);
}

static inline kernel_cap_t cap_raise_fs_set( kernel_cap_t a,
         kernel_cap_t permitted)
{
 kernel_cap_t __cap_fs_set = ((kernel_cap_t){{ ((1 << ((0) & 31)) | (1 << ((27) & 31)) | (1 << ((1) & 31)) | (1 << ((2) & 31)) | (1 << ((3) & 31)) | (1 << ((4) & 31))) | (1 << ((9) & 31)), ((1 << ((32) & 31))) } });
 return cap_combine(a,
      cap_intersect(permitted, __cap_fs_set));
}

static inline kernel_cap_t cap_drop_nfsd_set( kernel_cap_t a)
{
 kernel_cap_t __cap_fs_set = ((kernel_cap_t){{ ((1 << ((0) & 31)) | (1 << ((27) & 31)) | (1 << ((1) & 31)) | (1 << ((2) & 31)) | (1 << ((3) & 31)) | (1 << ((4) & 31))) | (1 << ((24) & 31)), ((1 << ((32) & 31))) } });
 return cap_drop(a, __cap_fs_set);
}

static inline kernel_cap_t cap_raise_nfsd_set( kernel_cap_t a,
           kernel_cap_t permitted)
{
 kernel_cap_t __cap_nfsd_set = ((kernel_cap_t){{ ((1 << ((0) & 31)) | (1 << ((27) & 31)) | (1 << ((1) & 31)) | (1 << ((2) & 31)) | (1 << ((3) & 31)) | (1 << ((4) & 31))) | (1 << ((24) & 31)), ((1 << ((32) & 31))) } });
 return cap_combine(a,
      cap_intersect(permitted, __cap_nfsd_set));
}

extern kernel_cap_t __cap_empty_set;
extern kernel_cap_t __cap_full_set;
extern kernel_cap_t __cap_init_eff_set;

kernel_cap_t cap_set_effective( kernel_cap_t pE_new);
extern int capable(int cap);
typedef __builtin_va_list __gnuc_va_list;
typedef __gnuc_va_list va_list;



















extern char * x86_cap_flags[8*32];
extern char * x86_power_flags[32];

struct alt_instr {
 u8 *instr;
 u8 *replacement;
 u8 cpuid;
 u8 instrlen;
 u8 replacementlen;
 u8 pad1;

 u32 pad2;

};

extern void alternative_instructions(void);
extern void apply_alternatives(struct alt_instr *start, struct alt_instr *end);

struct module;


extern void alternatives_smp_module_add(struct module *mod, char *name,
     void *locks, void *locks_end,
     void *text, void *text_end);
extern void alternatives_smp_module_del(struct module *mod);
extern void alternatives_smp_switch(int smp);
 unsigned char * *find_nop_table(void);
struct paravirt_patch_site;

void apply_paravirt(struct paravirt_patch_site *start,
      struct paravirt_patch_site *end);
extern void add_nops(void *insns, unsigned int len);
extern void *text_poke(void *addr, void *opcode, size_t len);
extern void *text_poke_early(void *addr, void *opcode, size_t len);
static inline void set_bit(unsigned int nr, unsigned long *addr)
{
 if ((__builtin_constant_p(nr))) {
  asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "orb %1,%0"
   : "+m" (*( long *) ((void *)(addr) + ((nr)>>3)))
   : "iq" ((u8)(1 << ((nr) & 7)))
   : "memory");
 } else {
  asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "bts %1,%0"
   : "+m" (*( long *) (addr)) : "Ir" (nr) : "memory");
 }
}
static inline void __set_bit(int nr, unsigned long *addr)
{
 asm ("bts %1,%0" : "+m" (*( long *) (addr)) : "Ir" (nr) : "memory");
}
static inline void clear_bit(int nr, unsigned long *addr)
{
 if ((__builtin_constant_p(nr))) {
  asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "andb %1,%0"
   : "+m" (*( long *) ((void *)(addr) + ((nr)>>3)))
   : "iq" ((u8)~(1 << ((nr) & 7))));
 } else {
  asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "btr %1,%0"
   : "+m" (*( long *) (addr))
   : "Ir" (nr));
 }
}
static inline void clear_bit_unlock(unsigned nr, unsigned long *addr)
{
 __asm__ __volatile__("": : :"memory");
 clear_bit(nr, addr);
}

static inline void __clear_bit(int nr, unsigned long *addr)
{
 asm ("btr %1,%0" : "+m" (*( long *) (addr)) : "Ir" (nr));
}
static inline void __clear_bit_unlock(unsigned nr, unsigned long *addr)
{
 __asm__ __volatile__("": : :"memory");
 __clear_bit(nr, addr);
}
static inline void __change_bit(int nr, unsigned long *addr)
{
 asm ("btc %1,%0" : "+m" (*( long *) (addr)) : "Ir" (nr));
}
static inline void change_bit(int nr, unsigned long *addr)
{
 asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "btc %1,%0" : "+m" (*( long *) (addr)) : "Ir" (nr));
}
static inline int test_and_set_bit(int nr, unsigned long *addr)
{
 int oldbit;

 asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "bts %2,%1\n\t"
       "sbb %0,%0" : "=r" (oldbit), "+m" (*( long *) (addr)) : "Ir" (nr) : "memory");

 return oldbit;
}
static inline int test_and_set_bit_lock(int nr, unsigned long *addr)
{
 return test_and_set_bit(nr, addr);
}
static inline int __test_and_set_bit(int nr, unsigned long *addr)
{
 int oldbit;

 asm("bts %2,%1\n\t"
     "sbb %0,%0"
     : "=r" (oldbit), "+m" (*( long *) (addr))
     : "Ir" (nr));
 return oldbit;
}
static inline int test_and_clear_bit(int nr, unsigned long *addr)
{
 int oldbit;

 asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "btr %2,%1\n\t"
       "sbb %0,%0"
       : "=r" (oldbit), "+m" (*( long *) (addr)) : "Ir" (nr) : "memory");

 return oldbit;
}
static inline int __test_and_clear_bit(int nr, unsigned long *addr)
{
 int oldbit;

 asm ("btr %2,%1\n\t"
       "sbb %0,%0"
       : "=r" (oldbit), "+m" (*( long *) (addr))
       : "Ir" (nr));
 return oldbit;
}


static inline int __test_and_change_bit(int nr, unsigned long *addr)
{
 int oldbit;

 asm ("btc %2,%1\n\t"
       "sbb %0,%0"
       : "=r" (oldbit), "+m" (*( long *) (addr))
       : "Ir" (nr) : "memory");

 return oldbit;
}
static inline int test_and_change_bit(int nr, unsigned long *addr)
{
 int oldbit;

 asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "btc %2,%1\n\t"
       "sbb %0,%0"
       : "=r" (oldbit), "+m" (*( long *) (addr)) : "Ir" (nr) : "memory");

 return oldbit;
}

static inline int constant_test_bit(int nr, unsigned long *addr)
{
 return ((1UL << (nr % 64)) &
  (((unsigned long *)addr)[nr / 64])) != 0;
}

static inline int variable_test_bit(int nr, unsigned long *addr)
{
 int oldbit;

 asm ("bt %2,%1\n\t"
       "sbb %0,%0"
       : "=r" (oldbit)
       : "m" (*(unsigned long *)addr), "Ir" (nr));

 return oldbit;
}
static inline unsigned long __ffs(unsigned long word)
{
 asm("bsf %1,%0"
  : "=r" (word)
  : "rm" (word));
 return word;
}







static inline unsigned long ffz(unsigned long word)
{
 asm("bsf %1,%0"
  : "=r" (word)
  : "r" (~word));
 return word;
}







static inline unsigned long __fls(unsigned long word)
{
 asm("bsr %1,%0"
     : "=r" (word)
     : "rm" (word));
 return word;
}
static inline int ffs(int x)
{
 int r;

 asm("bsfl %1,%0\n\t"
     "cmovzl %2,%0"
     : "=r" (r) : "rm" (x), "r" (-1));






 return r + 1;
}
static inline int fls(int x)
{
 int r;

 asm("bsrl %1,%0\n\t"
     "cmovzl %2,%0"
     : "=&r" (r) : "rm" (x), "rm" (-1));






 return r + 1;
}




static inline void set_bit_string(unsigned long *bitmap,
  unsigned long i, int len)
{
 unsigned long end = i + len;
 while (i < end) {
  __set_bit(i, bitmap);
  i++;
 }
}



static inline int sched_find_first_bit( unsigned long *b)
{

 if (b[0])
  return __ffs(b[0]);
 return __ffs(b[1]) + 64;
}








extern unsigned int hweight32(unsigned int w);
extern unsigned int hweight16(unsigned int w);
extern unsigned int hweight8(unsigned int w);
extern unsigned long hweight64(__u64 w);



static inline int fls64(__u64 x)
{
 if (x == 0)
  return 0;
 return __fls(x) + 1;
}










static inline __attribute__((__const__)) __u64 ___arch__swab64(__u64 x)
{
 asm("bswapq %0"
     : "=r" (x)
     : "0" (x));
 return x;
}

static inline __attribute__((__const__)) __u32 ___arch__swab32(__u32 x)
{
 asm("bswapl %0"
     : "=r" (x)
     : "0" (x));
 return x;
}
static __inline__ __attribute__((__const__)) __u16 ___swab16(__u16 x)
{
 return x<<8 | x>>8;
}
static __inline__ __attribute__((__const__)) __u32 ___swab32(__u32 x)
{
 return x<<24 | x>>24 |
  (x & (__u32)0x0000ff00UL)<<8 |
  (x & (__u32)0x00ff0000UL)>>8;
}
static __inline__ __attribute__((__const__)) __u64 ___swab64(__u64 x)
{
 return x<<56 | x>>56 |
  (x & (__u64)0x000000000000ff00ULL)<<40 |
  (x & (__u64)0x0000000000ff0000ULL)<<24 |
  (x & (__u64)0x00000000ff000000ULL)<< 8 |
         (x & (__u64)0x000000ff00000000ULL)>> 8 |
  (x & (__u64)0x0000ff0000000000ULL)>>24 |
  (x & (__u64)0x00ff000000000000ULL)>>40;
}
static __inline__ __attribute__((__const__)) __u16 __fswab16(__u16 x)
{
 return ___swab16(x);
}
static __inline__ __u16 __swab16p( __u16 *x)
{
 return ___swab16(*(x));
}
static __inline__ void __swab16s(__u16 *addr)
{
 ((void)(*(addr) = ___swab16(*(addr))));
}

static __inline__ __attribute__((__const__)) __u32 __fswab32(__u32 x)
{
 return ___arch__swab32(x);
}
static __inline__ __u32 __swab32p( __u32 *x)
{
 return ___arch__swab32(*(x));
}
static __inline__ void __swab32s(__u32 *addr)
{
 ((void)(*(addr) = ___arch__swab32(*(addr))));
}


static __inline__ __attribute__((__const__)) __u64 __fswab64(__u64 x)
{





 return ___arch__swab64(x);

}
static __inline__ __u64 __swab64p( __u64 *x)
{
 return ___arch__swab64(*(x));
}
static __inline__ void __swab64s(__u64 *addr)
{
 ((void)(*(addr) = ___arch__swab64(*(addr))));
}
static inline __le64 __cpu_to_le64p( __u64 *p)
{
 return ( __le64)*p;
}
static inline __u64 __le64_to_cpup( __le64 *p)
{
 return ( __u64)*p;
}
static inline __le32 __cpu_to_le32p( __u32 *p)
{
 return ( __le32)*p;
}
static inline __u32 __le32_to_cpup( __le32 *p)
{
 return ( __u32)*p;
}
static inline __le16 __cpu_to_le16p( __u16 *p)
{
 return ( __le16)*p;
}
static inline __u16 __le16_to_cpup( __le16 *p)
{
 return ( __u16)*p;
}
static inline __be64 __cpu_to_be64p( __u64 *p)
{
 return ( __be64)__swab64p(p);
}
static inline __u64 __be64_to_cpup( __be64 *p)
{
 return __swab64p((__u64 *)p);
}
static inline __be32 __cpu_to_be32p( __u32 *p)
{
 return ( __be32)__swab32p(p);
}
static inline __u32 __be32_to_cpup( __be32 *p)
{
 return __swab32p((__u32 *)p);
}
static inline __be16 __cpu_to_be16p( __u16 *p)
{
 return ( __be16)__swab16p(p);
}
static inline __u16 __be16_to_cpup( __be16 *p)
{
 return __swab16p((__u16 *)p);
}
static inline void le16_add_cpu(__le16 *var, u16 val)
{
 *var = (( __le16)(__u16)((( __u16)(__le16)(*var)) + val));
}

static inline void le32_add_cpu(__le32 *var, u32 val)
{
 *var = (( __le32)(__u32)((( __u32)(__le32)(*var)) + val));
}

static inline void le64_add_cpu(__le64 *var, u64 val)
{
 *var = (( __le64)(__u64)((( __u64)(__le64)(*var)) + val));
}

static inline void be16_add_cpu(__be16 *var, u16 val)
{
 *var = (( __be16)__fswab16((__fswab16(( __u16)(__be16)(*var)) + val)));
}

static inline void be32_add_cpu(__be32 *var, u32 val)
{
 *var = (( __be32)__fswab32((__fswab32(( __u32)(__be32)(*var)) + val)));
}

static inline void be64_add_cpu(__be64 *var, u64 val)
{
 *var = (( __be64)__fswab64((__fswab64(( __u64)(__be64)(*var)) + val)));
}













static __inline__ int get_bitmask_order(unsigned int count)
{
 int order;

 order = fls(count);
 return order;
}

static __inline__ int get_count_order(unsigned int count)
{
 int order;

 order = fls(count) - 1;
 if (count & (count - 1))
  order++;
 return order;
}

static inline unsigned long hweight_long(unsigned long w)
{
 return sizeof(w) == 4 ? hweight32(w) : hweight64(w);
}






static inline __u32 rol32(__u32 word, unsigned int shift)
{
 return (word << shift) | (word >> (32 - shift));
}






static inline __u32 ror32(__u32 word, unsigned int shift)
{
 return (word >> shift) | (word << (32 - shift));
}






static inline __u16 rol16(__u16 word, unsigned int shift)
{
 return (word << shift) | (word >> (16 - shift));
}






static inline __u16 ror16(__u16 word, unsigned int shift)
{
 return (word >> shift) | (word << (16 - shift));
}






static inline __u8 rol8(__u8 word, unsigned int shift)
{
 return (word << shift) | (word >> (8 - shift));
}






static inline __u8 ror8(__u8 word, unsigned int shift)
{
 return (word >> shift) | (word << (8 - shift));
}

static inline unsigned fls_long(unsigned long l)
{
 if (sizeof(l) == 4)
  return fls(l);
 return fls64(l);
}
extern unsigned long find_first_bit( unsigned long *addr,
        unsigned long size);
extern unsigned long find_first_zero_bit( unsigned long *addr,
      unsigned long size);
extern unsigned long find_next_bit( unsigned long *addr,
       unsigned long size, unsigned long offset);
extern unsigned long find_next_zero_bit( unsigned long *addr,
     unsigned long size,
     unsigned long offset);
extern __attribute__((, noreturn))
int ____ilog2_NaN(void);
static inline __attribute__(())
int __ilog2_u32(u32 n)
{
 return fls(n) - 1;
}



static inline __attribute__(())
int __ilog2_u64(u64 n)
{
 return fls64(n) - 1;
}







static inline __attribute__(())
bool is_power_of_2(unsigned long n)
{
 return (n != 0 && ((n & (n - 1)) == 0));
}




static inline __attribute__(())
unsigned long __roundup_pow_of_two(unsigned long n)
{
 return 1UL << fls_long(n - 1);
}




static inline __attribute__(())
unsigned long __rounddown_pow_of_two(unsigned long n)
{
 return 1UL << (fls_long(n) - 1);
}






struct ratelimit_state {
 int interval;
 int burst;
 int printed;
 int missed;
 unsigned long begin;
};




extern int __ratelimit(struct ratelimit_state *rs);

static inline int ratelimit(void)
{
 static struct ratelimit_state rs = {(5 * 1000), 10,}
                             ;
 return __ratelimit(&rs);
}


extern char linux_banner[];
extern char linux_proc_banner[];
extern int console_printk[];






struct completion;
struct pt_regs;
struct user;


extern int _cond_resched(void);
  void __might_sleep(char *file, int line);
extern struct atomic_notifier_head panic_notifier_list;
extern long (*panic_blink)(long time);
 void panic( char * fmt, ...)
 __attribute__ ((noreturn, format (printf, 1, 2))) __attribute__((__cold__));
extern void oops_enter(void);
extern void oops_exit(void);
extern int oops_may_print(void);
 void do_exit(long error_code)
 __attribute__((noreturn));
 void complete_and_exit(struct completion *, long)
 __attribute__((noreturn));
extern unsigned long simple_strtoul( char *,char **,unsigned int);
extern long simple_strtol( char *,char **,unsigned int);
extern unsigned long long simple_strtoull( char *,char **,unsigned int);
extern long long simple_strtoll( char *,char **,unsigned int);
extern int strict_strtoul( char *, unsigned int, unsigned long *);
extern int strict_strtol( char *, unsigned int, long *);
extern int strict_strtoull( char *, unsigned int, unsigned long long *);
extern int strict_strtoll( char *, unsigned int, long long *);
extern int sprintf(char * buf, char * fmt, ...)
 __attribute__ ((format (printf, 2, 3)));
extern int vsprintf(char *buf, char *, va_list)
 __attribute__ ((format (printf, 2, 0)));
extern int snprintf(char * buf, size_t size, char * fmt, ...)
 __attribute__ ((format (printf, 3, 4)));
extern int vsnprintf(char *buf, size_t size, char *fmt, va_list args)
 __attribute__ ((format (printf, 3, 0)));
extern int scnprintf(char * buf, size_t size, char * fmt, ...)
 __attribute__ ((format (printf, 3, 4)));
extern int vscnprintf(char *buf, size_t size, char *fmt, va_list args)
 __attribute__ ((format (printf, 3, 0)));
extern char *kasprintf(gfp_t gfp, char *fmt, ...)
 __attribute__ ((format (printf, 2, 3)));
extern char *kvasprintf(gfp_t gfp, char *fmt, va_list args);

extern int sscanf( char *, char *, ...)
 __attribute__ ((format (scanf, 2, 3)));
extern int vsscanf( char *, char *, va_list)
 __attribute__ ((format (scanf, 2, 0)));

extern int get_option(char **str, int *pint);
extern char *get_options( char *str, int nints, int *ints);
extern unsigned long long memparse(char *ptr, char **retptr);

extern int core_kernel_text(unsigned long addr);
extern int __kernel_text_address(unsigned long addr);
extern int kernel_text_address(unsigned long addr);
struct pid;
extern struct pid *session_of_pgrp(struct pid *pgrp);


 int vprintk( char *fmt, va_list args)
 __attribute__ ((format (printf, 1, 0)));
 int printk( char * fmt, ...)
 __attribute__ ((format (printf, 1, 2))) __attribute__((__cold__));

extern struct ratelimit_state printk_ratelimit_state;
extern int printk_ratelimit(void);
extern bool printk_timed_ratelimit(unsigned long *caller_jiffies,
       unsigned int interval_msec);
extern void __attribute__((format(printf, 1, 2)))
 early_printk( char *fmt, ...);

unsigned long int_sqrt(unsigned long);

static inline void console_silent(void)
{
 (console_printk[0]) = 0;
}

static inline void console_verbose(void)
{
 if ((console_printk[0]))
  (console_printk[0]) = 15;
}

extern void bust_spinlocks(int yes);
extern void wake_up_klogd(void);
extern int oops_in_progress;
extern int panic_timeout;
extern int panic_on_oops;
extern int panic_on_unrecovered_nmi;
extern int tainted;
extern char *print_tainted(void);
extern void add_taint(unsigned);
extern int root_mountflags;


extern enum system_states {
 SYSTEM_BOOTING,
 SYSTEM_RUNNING,
 SYSTEM_HALT,
 SYSTEM_POWER_OFF,
 SYSTEM_RESTART,
 SYSTEM_SUSPEND_DISK,
} system_state;
extern void dump_stack(void) __attribute__((__cold__));

enum {
 DUMP_PREFIX_NONE,
 DUMP_PREFIX_ADDRESS,
 DUMP_PREFIX_OFFSET
};
extern void hex_dump_to_buffer( void *buf, size_t len,
    int rowsize, int groupsize,
    char *linebuf, size_t linebuflen, bool ascii);
extern void print_hex_dump( char *level, char *prefix_str,
    int prefix_type, int rowsize, int groupsize,
    void *buf, size_t len, bool ascii);
extern void print_hex_dump_bytes( char *prefix_str, int prefix_type,
   void *buf, size_t len);

extern char hex_asc[];



static inline char *pack_hex_byte(char *buf, u8 byte)
{
 *buf++ = hex_asc[((byte) & 0xf0) >> 4];
 *buf++ = hex_asc[((byte) & 0x0f)];
 return buf;
}
struct sysinfo;
extern int do_sysinfo(struct sysinfo *info);




struct sysinfo {
 long uptime;
 unsigned long loads[3];
 unsigned long totalram;
 unsigned long freeram;
 unsigned long sharedram;
 unsigned long bufferram;
 unsigned long totalswap;
 unsigned long freeswap;
 unsigned short procs;
 unsigned short pad;
 unsigned long totalhigh;
 unsigned long freehigh;
 unsigned int mem_unit;
 char _f[20-2*sizeof(long)-sizeof(int)];
};











struct timespec;
struct compat_timespec;




struct restart_block {
 long (*fn)(struct restart_block *);
 union {
  struct {
   unsigned long arg0, arg1, arg2, arg3;
  };

  struct {
   u32 *uaddr;
   u32 val;
   u32 flags;
   u32 bitset;
   u64 time;
  } futex;

  struct {
   clockid_t index;
   struct timespec *rmtp;

   struct compat_timespec *compat_rmtp;

   u64 expires;
  } nanosleep;
 };
};

extern long do_no_restart_syscall(struct restart_block *parm);





void clear_page(void *page);
void copy_page(void *to, void *from);


extern unsigned long max_pfn;
extern unsigned long phys_base;

extern unsigned long __phys_addr(unsigned long);





typedef unsigned long pteval_t;
typedef unsigned long pmdval_t;
typedef unsigned long pudval_t;
typedef unsigned long pgdval_t;
typedef unsigned long pgprotval_t;
typedef unsigned long phys_addr_t;

typedef struct page *pgtable_t;

typedef struct { pteval_t pte; } pte_t;



extern unsigned long init_memory_mapping(unsigned long start,
      unsigned long end);

extern void initmem_init(unsigned long start_pfn, unsigned long end_pfn);

extern void init_extra_mapping_uc(unsigned long phys, unsigned long size);
extern void init_extra_mapping_wb(unsigned long phys, unsigned long size);
typedef struct { pgdval_t pgd; } pgd_t;
typedef struct { pgprotval_t pgprot; } pgprot_t;

extern int page_is_ram(unsigned long pagenr);
extern int devmem_is_allowed(unsigned long pagenr);
extern void map_devmem(unsigned long pfn, unsigned long size,
         pgprot_t vma_prot);
extern void unmap_devmem(unsigned long pfn, unsigned long size,
    pgprot_t vma_prot);

extern unsigned long max_low_pfn_mapped;
extern unsigned long max_pfn_mapped;

struct page;

static inline void clear_user_page(void *page, unsigned long vaddr,
    struct page *pg)
{
 clear_page(page);
}

static inline void copy_user_page(void *to, void *from, unsigned long vaddr,
    struct page *topage)
{
 copy_page(to, from);
}





static inline pgd_t native_make_pgd(pgdval_t val)
{

        return ({ pgd_t var; var.pgd = val; var; });
}

static inline pgdval_t native_pgd_val(pgd_t pgd)
{
 return pgd.pgd;
}



typedef struct { pudval_t pud; } pud_t;

static inline pud_t native_make_pud(pmdval_t val)
{

        return ({ pud_t var; var.pud = val; var; });
}

static inline pudval_t native_pud_val(pud_t pud)
{
 return pud.pud;
}
typedef struct { pmdval_t pmd; } pmd_t;

static inline pmd_t native_make_pmd(pmdval_t val)
{

        return ({ pmd_t var; var.pmd = val; var; });
}

static inline pmdval_t native_pmd_val(pmd_t pmd)
{
 return pmd.pmd;
}
static inline pte_t native_make_pte(pteval_t val)
{

        return ({ pte_t var; var.pte = val; var; });
}

static inline pteval_t native_pte_val(pte_t pte)
{
 return pte.pte;
}

static inline pteval_t native_pte_flags(pte_t pte)
{
 return native_pte_val(pte) & (~((pteval_t)(((signed long)(~(((1UL) << 12)-1))) & ((phys_addr_t)(1ULL << 46) - 1))));
}











struct desc_struct {
 union {
  struct {
   unsigned int a;
   unsigned int b;
  };
  struct {
   u16 limit0;
   u16 base0;
   unsigned base1: 8, type: 4, s: 1, dpl: 2, p: 1;
   unsigned limit: 4, avl: 1, l: 1, d: 1, g: 1, base2: 8;
  };
 };
} __attribute__((packed));

enum {
 GATE_INTERRUPT = 0xE,
 GATE_TRAP = 0xF,
 GATE_CALL = 0xC,
 GATE_TASK = 0x5,
};


struct gate_struct64 {
 u16 offset_low;
 u16 segment;
 unsigned ist : 3, zero0 : 5, type : 5, dpl : 2, p : 1;
 u16 offset_middle;
 u32 offset_high;
 u32 zero1;
} __attribute__((packed));





enum {
 DESC_TSS = 0x9,
 DESC_LDT = 0x2,
 DESCTYPE_S = 0x10,
};


struct ldttss_desc64 {
 u16 limit0;
 u16 base0;
 unsigned base1 : 8, type : 5, dpl : 2, p : 1;
 unsigned limit1 : 4, zero0 : 3, g : 1, base2 : 8;
 u32 base3;
 u32 zero1;
} __attribute__((packed));


typedef struct gate_struct64 gate_desc;
typedef struct ldttss_desc64 ldt_desc;
typedef struct ldttss_desc64 tss_desc;
struct desc_ptr {
 unsigned short size;
 unsigned long address;
} __attribute__((packed)) ;














extern char *strndup_user( char *, long);







static inline __attribute__((always_inline)) void *__inline_memcpy(void *to, void *from, size_t n)
{
 unsigned long d0, d1, d2;
 asm ("rep ; movsl\n\t"
       "testb $2,%b4\n\t"
       "je 1f\n\t"
       "movsw\n"
       "1:\ttestb $1,%b4\n\t"
       "je 2f\n\t"
       "movsb\n"
       "2:"
       : "=&c" (d0), "=&D" (d1), "=&S" (d2)
       : "0" (n / 4), "q" (n), "1" ((long)to), "2" ((long)from)
       : "memory");
 return to;
}






extern void *memcpy(void *to, void *from, size_t len);
void *memset(void *s, int c, size_t n);


void *memmove(void *dest, void *src, size_t count);

int memcmp( void *cs, void *ct, size_t count);
size_t strlen( char *s);
char *strcpy(char *dest, char *src);
char *strcat(char *dest, char *src);
int strcmp( char *cs, char *ct);


extern char * strcpy(char *, char *);


extern char * strncpy(char *, char *, __kernel_size_t);


size_t strlcpy(char *, char *, size_t);


extern char * strcat(char *, char *);


extern char * strncat(char *, char *, __kernel_size_t);


extern size_t strlcat(char *, char *, __kernel_size_t);


extern int strcmp( char *, char *);


extern int strncmp( char *, char *,__kernel_size_t);


extern int strnicmp( char *, char *, __kernel_size_t);


extern int strcasecmp( char *s1, char *s2);


extern int strncasecmp( char *s1, char *s2, size_t n);


extern char * strchr( char *,int);


extern char * strnchr( char *, size_t, int);


extern char * strrchr( char *,int);

extern char * strstrip(char *);

extern char * strstr( char *, char *);


extern __kernel_size_t strlen( char *);


extern __kernel_size_t strnlen( char *,__kernel_size_t);


extern char * strpbrk( char *, char *);


extern char * strsep(char **, char *);


extern __kernel_size_t strspn( char *, char *);


extern __kernel_size_t strcspn( char *, char *);
extern void * memscan(void *,int,__kernel_size_t);


extern int memcmp( void *, void *,__kernel_size_t);


extern void * memchr( void *,int,__kernel_size_t);


extern char *kstrdup( char *s, gfp_t gfp);
extern char *kstrndup( char *s, size_t len, gfp_t gfp);
extern void *kmemdup( void *src, size_t len, gfp_t gfp);

extern char **argv_split(gfp_t gfp, char *str, int *argcp);
extern void argv_free(char **argv);

extern bool sysfs_streq( char *s1, char *s2);

extern ssize_t memory_read_from_buffer(void *to, size_t count, loff_t *ppos,
   void *from, size_t available);
extern int __bitmap_empty( unsigned long *bitmap, int bits);
extern int __bitmap_full( unsigned long *bitmap, int bits);
extern int __bitmap_equal( unsigned long *bitmap1,
                 unsigned long *bitmap2, int bits);
extern void __bitmap_complement(unsigned long *dst, unsigned long *src,
   int bits);
extern void __bitmap_shift_right(unsigned long *dst,
                        unsigned long *src, int shift, int bits);
extern void __bitmap_shift_left(unsigned long *dst,
                        unsigned long *src, int shift, int bits);
extern void __bitmap_and(unsigned long *dst, unsigned long *bitmap1,
   unsigned long *bitmap2, int bits);
extern void __bitmap_or(unsigned long *dst, unsigned long *bitmap1,
   unsigned long *bitmap2, int bits);
extern void __bitmap_xor(unsigned long *dst, unsigned long *bitmap1,
   unsigned long *bitmap2, int bits);
extern void __bitmap_andnot(unsigned long *dst, unsigned long *bitmap1,
   unsigned long *bitmap2, int bits);
extern int __bitmap_intersects( unsigned long *bitmap1,
   unsigned long *bitmap2, int bits);
extern int __bitmap_subset( unsigned long *bitmap1,
   unsigned long *bitmap2, int bits);
extern int __bitmap_weight( unsigned long *bitmap, int bits);

extern int bitmap_scnprintf(char *buf, unsigned int len,
   unsigned long *src, int nbits);
extern int bitmap_scnprintf_len(unsigned int nr_bits);
extern int __bitmap_parse( char *buf, unsigned int buflen, int is_user,
   unsigned long *dst, int nbits);
extern int bitmap_parse_user( char *ubuf, unsigned int ulen,
   unsigned long *dst, int nbits);
extern int bitmap_scnlistprintf(char *buf, unsigned int len,
   unsigned long *src, int nbits);
extern int bitmap_parselist( char *buf, unsigned long *maskp,
   int nmaskbits);
extern void bitmap_remap(unsigned long *dst, unsigned long *src,
  unsigned long *old, unsigned long *_new, int bits);
extern int bitmap_bitremap(int oldbit,
  unsigned long *old, unsigned long *_new, int bits);
extern void bitmap_onto(unsigned long *dst, unsigned long *orig,
  unsigned long *relmap, int bits);
extern void bitmap_fold(unsigned long *dst, unsigned long *orig,
  int sz, int bits);
extern int bitmap_find_free_region(unsigned long *bitmap, int bits, int order);
extern void bitmap_release_region(unsigned long *bitmap, int pos, int order);
extern int bitmap_allocate_region(unsigned long *bitmap, int pos, int order);







static inline void bitmap_zero(unsigned long *dst, int nbits)
{
 if (nbits <= 64)
  *dst = 0UL;
 else {
  int len = (((nbits) + (8 * sizeof(long)) - 1) / (8 * sizeof(long))) * sizeof(unsigned long);
  memset(dst, 0, len);
 }
}

static inline void bitmap_fill(unsigned long *dst, int nbits)
{
 size_t nlongs = (((nbits) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)));
 if (nlongs > 1) {
  int len = (nlongs - 1) * sizeof(unsigned long);
  memset(dst, 0xff, len);
 }
 dst[nlongs - 1] = ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL );
}

static inline void bitmap_copy(unsigned long *dst, unsigned long *src,
   int nbits)
{
 if (nbits <= 64)
  *dst = *src;
 else {
  int len = (((nbits) + (8 * sizeof(long)) - 1) / (8 * sizeof(long))) * sizeof(unsigned long);
  memcpy(dst, src, len);
 }
}

static inline void bitmap_and(unsigned long *dst, unsigned long *src1,
   unsigned long *src2, int nbits)
{
 if (nbits <= 64)
  *dst = *src1 & *src2;
 else
  __bitmap_and(dst, src1, src2, nbits);
}

static inline void bitmap_or(unsigned long *dst, unsigned long *src1,
   unsigned long *src2, int nbits)
{
 if (nbits <= 64)
  *dst = *src1 | *src2;
 else
  __bitmap_or(dst, src1, src2, nbits);
}

static inline void bitmap_xor(unsigned long *dst, unsigned long *src1,
   unsigned long *src2, int nbits)
{
 if (nbits <= 64)
  *dst = *src1 ^ *src2;
 else
  __bitmap_xor(dst, src1, src2, nbits);
}

static inline void bitmap_andnot(unsigned long *dst, unsigned long *src1,
   unsigned long *src2, int nbits)
{
 if (nbits <= 64)
  *dst = *src1 & ~(*src2);
 else
  __bitmap_andnot(dst, src1, src2, nbits);
}

static inline void bitmap_complement(unsigned long *dst, unsigned long *src,
   int nbits)
{
 if (nbits <= 64)
  *dst = ~(*src) & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL );
 else
  __bitmap_complement(dst, src, nbits);
}

static inline int bitmap_equal( unsigned long *src1,
   unsigned long *src2, int nbits)
{
 if (nbits <= 64)
  return ! ((*src1 ^ *src2) & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL ));
 else
  return __bitmap_equal(src1, src2, nbits);
}

static inline int bitmap_intersects( unsigned long *src1,
   unsigned long *src2, int nbits)
{
 if (nbits <= 64)
  return ((*src1 & *src2) & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL )) != 0;
 else
  return __bitmap_intersects(src1, src2, nbits);
}

static inline int bitmap_subset( unsigned long *src1,
   unsigned long *src2, int nbits)
{
 if (nbits <= 64)
  return ! ((*src1 & ~(*src2)) & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL ));
 else
  return __bitmap_subset(src1, src2, nbits);
}

static inline int bitmap_empty( unsigned long *src, int nbits)
{
 if (nbits <= 64)
  return ! (*src & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL ));
 else
  return __bitmap_empty(src, nbits);
}

static inline int bitmap_full( unsigned long *src, int nbits)
{
 if (nbits <= 64)
  return ! (~(*src) & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL ));
 else
  return __bitmap_full(src, nbits);
}

static inline int bitmap_weight( unsigned long *src, int nbits)
{
 if (nbits <= 64)
  return hweight_long(*src & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL ));
 return __bitmap_weight(src, nbits);
}

static inline void bitmap_shift_right(unsigned long *dst,
   unsigned long *src, int n, int nbits)
{
 if (nbits <= 64)
  *dst = *src >> n;
 else
  __bitmap_shift_right(dst, src, n, nbits);
}

static inline void bitmap_shift_left(unsigned long *dst,
   unsigned long *src, int n, int nbits)
{
 if (nbits <= 64)
  *dst = (*src << n) & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL );
 else
  __bitmap_shift_left(dst, src, n, nbits);
}

static inline int bitmap_parse( char *buf, unsigned int buflen,
   unsigned long *maskp, int nmaskbits)
{
 return __bitmap_parse(buf, buflen, 0, maskp, nmaskbits);
}

typedef struct _cpumask { unsigned long bits[(((64) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))]; } cpumask_t;
extern cpumask_t _unused_cpumask_arg_;


static inline void __cpu_set(int cpu, cpumask_t *dstp)
{
 set_bit(cpu, dstp->bits);
}


static inline void __cpu_clear(int cpu, cpumask_t *dstp)
{
 clear_bit(cpu, dstp->bits);
}


static inline void __cpus_setall(cpumask_t *dstp, int nbits)
{
 bitmap_fill(dstp->bits, nbits);
}


static inline void __cpus_clear(cpumask_t *dstp, int nbits)
{
 bitmap_zero(dstp->bits, nbits);
}





static inline int __cpu_test_and_set(int cpu, cpumask_t *addr)
{
 return test_and_set_bit(cpu, addr->bits);
}


static inline void __cpus_and(cpumask_t *dstp, cpumask_t *src1p,
     cpumask_t *src2p, int nbits)
{
 bitmap_and(dstp->bits, src1p->bits, src2p->bits, nbits);
}


static inline void __cpus_or(cpumask_t *dstp, cpumask_t *src1p,
     cpumask_t *src2p, int nbits)
{
 bitmap_or(dstp->bits, src1p->bits, src2p->bits, nbits);
}


static inline void __cpus_xor(cpumask_t *dstp, cpumask_t *src1p,
     cpumask_t *src2p, int nbits)
{
 bitmap_xor(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline void __cpus_andnot(cpumask_t *dstp, cpumask_t *src1p,
     cpumask_t *src2p, int nbits)
{
 bitmap_andnot(dstp->bits, src1p->bits, src2p->bits, nbits);
}


static inline void __cpus_complement(cpumask_t *dstp,
     cpumask_t *srcp, int nbits)
{
 bitmap_complement(dstp->bits, srcp->bits, nbits);
}


static inline int __cpus_equal( cpumask_t *src1p,
     cpumask_t *src2p, int nbits)
{
 return bitmap_equal(src1p->bits, src2p->bits, nbits);
}


static inline int __cpus_intersects( cpumask_t *src1p,
     cpumask_t *src2p, int nbits)
{
 return bitmap_intersects(src1p->bits, src2p->bits, nbits);
}


static inline int __cpus_subset( cpumask_t *src1p,
     cpumask_t *src2p, int nbits)
{
 return bitmap_subset(src1p->bits, src2p->bits, nbits);
}


static inline int __cpus_empty( cpumask_t *srcp, int nbits)
{
 return bitmap_empty(srcp->bits, nbits);
}


static inline int __cpus_full( cpumask_t *srcp, int nbits)
{
 return bitmap_full(srcp->bits, nbits);
}


static inline int __cpus_weight( cpumask_t *srcp, int nbits)
{
 return bitmap_weight(srcp->bits, nbits);
}



static inline void __cpus_shift_right(cpumask_t *dstp,
     cpumask_t *srcp, int n, int nbits)
{
 bitmap_shift_right(dstp->bits, srcp->bits, n, nbits);
}



static inline void __cpus_shift_left(cpumask_t *dstp,
     cpumask_t *srcp, int n, int nbits)
{
 bitmap_shift_left(dstp->bits, srcp->bits, n, nbits);
}
extern unsigned long
 cpu_bit_bitmap[64 +1][(((64) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];

static inline cpumask_t *get_cpu_mask(unsigned int cpu)
{
 unsigned long *p = cpu_bit_bitmap[1 + cpu % 64];
 p -= cpu / 64;
 return ( cpumask_t *)p;
}
static inline int __cpumask_scnprintf(char *buf, int len,
     cpumask_t *srcp, int nbits)
{
 return bitmap_scnprintf(buf, len, srcp->bits, nbits);
}



static inline int __cpumask_parse_user( char *buf, int len,
     cpumask_t *dstp, int nbits)
{
 return bitmap_parse_user(buf, len, dstp->bits, nbits);
}



static inline int __cpulist_scnprintf(char *buf, int len,
     cpumask_t *srcp, int nbits)
{
 return bitmap_scnlistprintf(buf, len, srcp->bits, nbits);
}


static inline int __cpulist_parse( char *buf, cpumask_t *dstp, int nbits)
{
 return bitmap_parselist(buf, dstp->bits, nbits);
}



static inline int __cpu_remap(int oldbit,
  cpumask_t *oldp, cpumask_t *newp, int nbits)
{
 return bitmap_bitremap(oldbit, oldp->bits, newp->bits, nbits);
}



static inline void __cpus_remap(cpumask_t *dstp, cpumask_t *srcp,
  cpumask_t *oldp, cpumask_t *newp, int nbits)
{
 bitmap_remap(dstp->bits, srcp->bits, oldp->bits, newp->bits, nbits);
}



static inline void __cpus_onto(cpumask_t *dstp, cpumask_t *origp,
  cpumask_t *relmapp, int nbits)
{
 bitmap_onto(dstp->bits, origp->bits, relmapp->bits, nbits);
}



static inline void __cpus_fold(cpumask_t *dstp, cpumask_t *origp,
  int sz, int nbits)
{
 bitmap_fold(dstp->bits, origp->bits, sz, nbits);
}
extern int nr_cpu_ids;
int __first_cpu( cpumask_t *srcp);
int __next_cpu(int n, cpumask_t *srcp);
int __any_online_cpu( cpumask_t *mask);
extern cpumask_t cpu_possible_map;
extern cpumask_t cpu_online_map;
extern cpumask_t cpu_present_map;
extern cpumask_t cpu_active_map;
enum km_type {
 KM_BOUNCE_READ,
 KM_SKB_SUNRPC_DATA,
 KM_SKB_DATA_SOFTIRQ,
 KM_USER0,
 KM_USER1,
 KM_BIO_SRC_IRQ,
 KM_BIO_DST_IRQ,
 KM_PTE0,
 KM_PTE1,
 KM_IRQ0,
 KM_IRQ1,
 KM_SOFTIRQ0,
 KM_SOFTIRQ1,
 KM_TYPE_NR
};


struct page;
struct thread_struct;
struct desc_ptr;
struct tss_struct;
struct mm_struct;
struct desc_struct;


struct pv_info {
 unsigned int kernel_rpl;
 int shared_kernel_pmd;
 int paravirt_enabled;
 char *name;
};

struct pv_init_ops {
 unsigned (*patch)(u8 type, u16 clobber, void *insnbuf,
     unsigned long addr, unsigned len);


 void (*arch_setup)(void);
 char *(*memory_setup)(void);
 void (*post_allocator_init)(void);


 void (*banner)(void);
};


struct pv_lazy_ops {

 void (*enter)(void);
 void (*leave)(void);
};

struct pv_time_ops {
 void (*time_init)(void);


 unsigned long (*get_wallclock)(void);
 int (*set_wallclock)(unsigned long);

 unsigned long long (*sched_clock)(void);
 unsigned long (*get_tsc_khz)(void);
};

struct pv_cpu_ops {

 unsigned long (*get_debugreg)(int regno);
 void (*set_debugreg)(int regno, unsigned long value);

 void (*clts)(void);

 unsigned long (*read_cr0)(void);
 void (*write_cr0)(unsigned long);

 unsigned long (*read_cr4_safe)(void);
 unsigned long (*read_cr4)(void);
 void (*write_cr4)(unsigned long);


 unsigned long (*read_cr8)(void);
 void (*write_cr8)(unsigned long);



 void (*load_tr_desc)(void);
 void (*load_gdt)( struct desc_ptr *);
 void (*load_idt)( struct desc_ptr *);
 void (*store_gdt)(struct desc_ptr *);
 void (*store_idt)(struct desc_ptr *);
 void (*set_ldt)( void *desc, unsigned entries);
 unsigned long (*store_tr)(void);
 void (*load_tls)(struct thread_struct *t, unsigned int cpu);

 void (*load_gs_index)(unsigned int idx);

 void (*write_ldt_entry)(struct desc_struct *ldt, int entrynum,
    void *desc);
 void (*write_gdt_entry)(struct desc_struct *,
    int entrynum, void *desc, int size);
 void (*write_idt_entry)(gate_desc *,
    int entrynum, gate_desc *gate);
 void (*load_sp0)(struct tss_struct *tss, struct thread_struct *t);

 void (*set_iopl_mask)(unsigned mask);

 void (*wbinvd)(void);
 void (*io_delay)(void);


 void (*cpuid)(unsigned int *eax, unsigned int *ebx,
        unsigned int *ecx, unsigned int *edx);



 u64 (*read_msr)(unsigned int msr, int *err);
 int (*write_msr)(unsigned int msr, unsigned low, unsigned high);

 u64 (*read_tsc)(void);
 u64 (*read_pmc)(int counter);
 unsigned long long (*read_tscp)(unsigned int *aux);







 void (*irq_enable_sysexit)(void);







 void (*usergs_sysret64)(void);







 void (*usergs_sysret32)(void);



 void (*iret)(void);

 void (*swapgs)(void);

 struct pv_lazy_ops lazy_mode;
};

struct pv_irq_ops {
 void (*init_IRQ)(void);







 unsigned long (*save_fl)(void);
 void (*restore_fl)(unsigned long);
 void (*irq_disable)(void);
 void (*irq_enable)(void);
 void (*safe_halt)(void);
 void (*halt)(void);


 void (*adjust_exception_frame)(void);

};

struct pv_apic_ops {





 void (*apic_write)(unsigned long reg, u32 v);
 u32 (*apic_read)(unsigned long reg);
 void (*setup_boot_clock)(void);
 void (*setup_secondary_clock)(void);

 void (*startup_ipi_hook)(int phys_apicid,
     unsigned long start_eip,
     unsigned long start_esp);

};

struct pv_mmu_ops {






 void (*pagetable_setup_start)(pgd_t *pgd_base);
 void (*pagetable_setup_done)(pgd_t *pgd_base);

 unsigned long (*read_cr2)(void);
 void (*write_cr2)(unsigned long);

 unsigned long (*read_cr3)(void);
 void (*write_cr3)(unsigned long);





 void (*activate_mm)(struct mm_struct *prev,
       struct mm_struct *next);
 void (*dup_mmap)(struct mm_struct *oldmm,
    struct mm_struct *mm);
 void (*exit_mmap)(struct mm_struct *mm);



 void (*flush_tlb_user)(void);
 void (*flush_tlb_kernel)(void);
 void (*flush_tlb_single)(unsigned long addr);
 void (*flush_tlb_others)( cpumask_t *cpus, struct mm_struct *mm,
     unsigned long va);


 int (*pgd_alloc)(struct mm_struct *mm);
 void (*pgd_free)(struct mm_struct *mm, pgd_t *pgd);





 void (*alloc_pte)(struct mm_struct *mm, u32 pfn);
 void (*alloc_pmd)(struct mm_struct *mm, u32 pfn);
 void (*alloc_pmd_clone)(u32 pfn, u32 clonepfn, u32 start, u32 count);
 void (*alloc_pud)(struct mm_struct *mm, u32 pfn);
 void (*release_pte)(u32 pfn);
 void (*release_pmd)(u32 pfn);
 void (*release_pud)(u32 pfn);


 void (*set_pte)(pte_t *ptep, pte_t pteval);
 void (*set_pte_at)(struct mm_struct *mm, unsigned long addr,
      pte_t *ptep, pte_t pteval);
 void (*set_pmd)(pmd_t *pmdp, pmd_t pmdval);
 void (*pte_update)(struct mm_struct *mm, unsigned long addr,
      pte_t *ptep);
 void (*pte_update_defer)(struct mm_struct *mm,
     unsigned long addr, pte_t *ptep);

 pte_t (*ptep_modify_prot_start)(struct mm_struct *mm, unsigned long addr,
     pte_t *ptep);
 void (*ptep_modify_prot_commit)(struct mm_struct *mm, unsigned long addr,
     pte_t *ptep, pte_t pte);

 pteval_t (*pte_val)(pte_t);
 pteval_t (*pte_flags)(pte_t);
 pte_t (*make_pte)(pteval_t pte);

 pgdval_t (*pgd_val)(pgd_t);
 pgd_t (*make_pgd)(pgdval_t pgd);
 void (*set_pud)(pud_t *pudp, pud_t pudval);

 pmdval_t (*pmd_val)(pmd_t);
 pmd_t (*make_pmd)(pmdval_t pmd);


 pudval_t (*pud_val)(pud_t);
 pud_t (*make_pud)(pudval_t pud);

 void (*set_pgd)(pgd_t *pudp, pgd_t pgdval);







 struct pv_lazy_ops lazy_mode;





 void (*set_fixmap)(unsigned idx,
      unsigned long phys, pgprot_t flags);
};

struct raw_spinlock;
struct pv_lock_ops {
 int (*spin_is_locked)(struct raw_spinlock *lock);
 int (*spin_is_contended)(struct raw_spinlock *lock);
 void (*spin_lock)(struct raw_spinlock *lock);
 int (*spin_trylock)(struct raw_spinlock *lock);
 void (*spin_unlock)(struct raw_spinlock *lock);
};




struct paravirt_patch_template {
 struct pv_init_ops pv_init_ops;
 struct pv_time_ops pv_time_ops;
 struct pv_cpu_ops pv_cpu_ops;
 struct pv_irq_ops pv_irq_ops;
 struct pv_apic_ops pv_apic_ops;
 struct pv_mmu_ops pv_mmu_ops;
 struct pv_lock_ops pv_lock_ops;
};

extern struct pv_info pv_info;
extern struct pv_init_ops pv_init_ops;
extern struct pv_time_ops pv_time_ops;
extern struct pv_cpu_ops pv_cpu_ops;
extern struct pv_irq_ops pv_irq_ops;
extern struct pv_apic_ops pv_apic_ops;
extern struct pv_mmu_ops pv_mmu_ops;
extern struct pv_lock_ops pv_lock_ops;
unsigned paravirt_patch_nop(void);
unsigned paravirt_patch_ignore(unsigned len);
unsigned paravirt_patch_call(void *insnbuf,
        void *target, u16 tgt_clobbers,
        unsigned long addr, u16 site_clobbers,
        unsigned len);
unsigned paravirt_patch_jmp(void *insnbuf, void *target,
       unsigned long addr, unsigned len);
unsigned paravirt_patch_default(u8 type, u16 clobbers, void *insnbuf,
    unsigned long addr, unsigned len);

unsigned paravirt_patch_insns(void *insnbuf, unsigned len,
         char *start, char *end);

unsigned native_patch(u8 type, u16 clobbers, void *ibuf,
        unsigned long addr, unsigned len);

int paravirt_disable_iospace(void);
static inline int paravirt_enabled(void)
{
 return pv_info.paravirt_enabled;
}

static inline void load_sp0(struct tss_struct *tss,
        struct thread_struct *thread)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.load_sp0); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.load_sp0) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.load_sp0), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(tss)), "1" ((unsigned long)(thread)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}


static inline unsigned long get_wallclock(void)
{
 return ({ unsigned long __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_time_ops.get_wallclock); if (sizeof(unsigned long) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_time_ops.get_wallclock) / sizeof(void *))), [paravirt_opptr] "m" (pv_time_ops.get_wallclock), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_time_ops.get_wallclock) / sizeof(void *))), [paravirt_opptr] "m" (pv_time_ops.get_wallclock), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)__eax; } __ret; });
}

static inline int set_wallclock(unsigned long nowtime)
{
 return ({ int __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_time_ops.set_wallclock); if (sizeof(int) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_time_ops.set_wallclock) / sizeof(void *))), [paravirt_opptr] "m" (pv_time_ops.set_wallclock), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(nowtime)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (int)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_time_ops.set_wallclock) / sizeof(void *))), [paravirt_opptr] "m" (pv_time_ops.set_wallclock), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(nowtime)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (int)__eax; } __ret; });
}

static inline void (*choose_time_init(void))(void)
{
 return pv_time_ops.time_init;
}


static inline void __cpuid(unsigned int *eax, unsigned int *ebx,
      unsigned int *ecx, unsigned int *edx)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.cpuid); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.cpuid) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.cpuid), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(eax)), "1"((unsigned long)(ebx)), "2"((unsigned long)(ecx)), "3"((unsigned long)(edx)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}




static inline unsigned long paravirt_get_debugreg(int reg)
{
 return ({ unsigned long __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_cpu_ops.get_debugreg); if (sizeof(unsigned long) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.get_debugreg) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.get_debugreg), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(reg)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.get_debugreg) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.get_debugreg), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(reg)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)__eax; } __ret; });
}

static inline void set_debugreg(unsigned long val, int reg)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.set_debugreg); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.set_debugreg) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.set_debugreg), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(reg)), "1" ((unsigned long)(val)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void clts(void)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.clts); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.clts) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.clts), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline unsigned long read_cr0(void)
{
 return ({ unsigned long __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_cpu_ops.read_cr0); if (sizeof(unsigned long) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_cr0) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_cr0), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_cr0) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_cr0), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)__eax; } __ret; });
}

static inline void write_cr0(unsigned long x)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.write_cr0); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.write_cr0) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.write_cr0), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(x)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline unsigned long read_cr2(void)
{
 return ({ unsigned long __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.read_cr2); if (sizeof(unsigned long) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.read_cr2) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.read_cr2), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.read_cr2) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.read_cr2), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)__eax; } __ret; });
}

static inline void write_cr2(unsigned long x)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.write_cr2); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.write_cr2) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.write_cr2), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(x)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline unsigned long read_cr3(void)
{
 return ({ unsigned long __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.read_cr3); if (sizeof(unsigned long) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.read_cr3) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.read_cr3), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.read_cr3) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.read_cr3), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)__eax; } __ret; });
}

static inline void write_cr3(unsigned long x)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.write_cr3); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.write_cr3) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.write_cr3), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(x)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline unsigned long read_cr4(void)
{
 return ({ unsigned long __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_cpu_ops.read_cr4); if (sizeof(unsigned long) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_cr4) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_cr4), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_cr4) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_cr4), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)__eax; } __ret; });
}
static inline unsigned long read_cr4_safe(void)
{
 return ({ unsigned long __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_cpu_ops.read_cr4_safe); if (sizeof(unsigned long) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_cr4_safe) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_cr4_safe), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_cr4_safe) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_cr4_safe), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)__eax; } __ret; });
}

static inline void write_cr4(unsigned long x)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.write_cr4); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.write_cr4) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.write_cr4), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(x)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}


static inline unsigned long read_cr8(void)
{
 return ({ unsigned long __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_cpu_ops.read_cr8); if (sizeof(unsigned long) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_cr8) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_cr8), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_cr8) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_cr8), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)__eax; } __ret; });
}

static inline void write_cr8(unsigned long x)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.write_cr8); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.write_cr8) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.write_cr8), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(x)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}


static inline void raw_safe_halt(void)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_irq_ops.safe_halt); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_irq_ops.safe_halt) / sizeof(void *))), [paravirt_opptr] "m" (pv_irq_ops.safe_halt), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void halt(void)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_irq_ops.safe_halt); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_irq_ops.safe_halt) / sizeof(void *))), [paravirt_opptr] "m" (pv_irq_ops.safe_halt), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void wbinvd(void)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.wbinvd); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.wbinvd) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.wbinvd), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}



static inline u64 paravirt_read_msr(unsigned msr, int *err)
{
 return ({ u64 __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_cpu_ops.read_msr); if (sizeof(u64) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_msr) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_msr), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(msr)), "1" ((unsigned long)(err)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (u64)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_msr) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_msr), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(msr)), "1" ((unsigned long)(err)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (u64)__eax; } __ret; });
}
static inline int paravirt_write_msr(unsigned msr, unsigned low, unsigned high)
{
 return ({ int __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_cpu_ops.write_msr); if (sizeof(int) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.write_msr) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.write_msr), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(msr)), "1"((unsigned long)(low)), "2"((unsigned long)(high)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (int)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.write_msr) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.write_msr), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(msr)), "1"((unsigned long)(low)), "2"((unsigned long)(high)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (int)__eax; } __ret; });
}
static inline int rdmsrl_safe(unsigned msr, unsigned long long *p)
{
 int err;

 *p = paravirt_read_msr(msr, &err);
 return err;
}

static inline u64 paravirt_read_tsc(void)
{
 return ({ u64 __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_cpu_ops.read_tsc); if (sizeof(u64) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_tsc) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_tsc), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (u64)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_tsc) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_tsc), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (u64)__eax; } __ret; });
}
static inline unsigned long long paravirt_sched_clock(void)
{
 return ({ unsigned long long __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_time_ops.sched_clock); if (sizeof(unsigned long long) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_time_ops.sched_clock) / sizeof(void *))), [paravirt_opptr] "m" (pv_time_ops.sched_clock), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long long)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_time_ops.sched_clock) / sizeof(void *))), [paravirt_opptr] "m" (pv_time_ops.sched_clock), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long long)__eax; } __ret; });
}


static inline unsigned long long paravirt_read_pmc(int counter)
{
 return ({ u64 __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_cpu_ops.read_pmc); if (sizeof(u64) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_pmc) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_pmc), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(counter)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (u64)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_pmc) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_pmc), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(counter)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (u64)__eax; } __ret; });
}
static inline unsigned long long paravirt_rdtscp(unsigned int *aux)
{
 return ({ u64 __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_cpu_ops.read_tscp); if (sizeof(u64) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_tscp) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_tscp), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(aux)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (u64)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.read_tscp) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.read_tscp), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(aux)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (u64)__eax; } __ret; });
}
static inline void load_TR_desc(void)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.load_tr_desc); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.load_tr_desc) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.load_tr_desc), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}
static inline void load_gdt( struct desc_ptr *dtr)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.load_gdt); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.load_gdt) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.load_gdt), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(dtr)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}
static inline void load_idt( struct desc_ptr *dtr)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.load_idt); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.load_idt) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.load_idt), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(dtr)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}
static inline void set_ldt( void *addr, unsigned entries)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.set_ldt); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.set_ldt) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.set_ldt), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(addr)), "1" ((unsigned long)(entries)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}
static inline void store_gdt(struct desc_ptr *dtr)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.store_gdt); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.store_gdt) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.store_gdt), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(dtr)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}
static inline void store_idt(struct desc_ptr *dtr)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.store_idt); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.store_idt) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.store_idt), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(dtr)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}
static inline unsigned long paravirt_store_tr(void)
{
 return ({ unsigned long __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_cpu_ops.store_tr); if (sizeof(unsigned long) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.store_tr) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.store_tr), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.store_tr) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.store_tr), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)__eax; } __ret; });
}

static inline void load_TLS(struct thread_struct *t, unsigned cpu)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.load_tls); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.load_tls) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.load_tls), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(t)), "1" ((unsigned long)(cpu)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}


static inline void load_gs_index(unsigned int gs)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.load_gs_index); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.load_gs_index) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.load_gs_index), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(gs)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}


static inline void write_ldt_entry(struct desc_struct *dt, int entry,
       void *desc)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.write_ldt_entry); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.write_ldt_entry) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.write_ldt_entry), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(dt)), "1"((unsigned long)(entry)), "2"((unsigned long)(desc)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void write_gdt_entry(struct desc_struct *dt, int entry,
       void *desc, int type)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.write_gdt_entry); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.write_gdt_entry) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.write_gdt_entry), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(dt)), "1"((unsigned long)(entry)), "2"((unsigned long)(desc)), "3"((unsigned long)(type)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void write_idt_entry(gate_desc *dt, int entry, gate_desc *g)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.write_idt_entry); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.write_idt_entry) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.write_idt_entry), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(dt)), "1"((unsigned long)(entry)), "2"((unsigned long)(g)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}
static inline void set_iopl_mask(unsigned mask)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.set_iopl_mask); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.set_iopl_mask) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.set_iopl_mask), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(mask)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}


static inline void slow_down_io(void)
{
 pv_cpu_ops.io_delay();





}





static inline void apic_write(unsigned long reg, u32 v)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_apic_ops.apic_write); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_apic_ops.apic_write) / sizeof(void *))), [paravirt_opptr] "m" (pv_apic_ops.apic_write), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(reg)), "1" ((unsigned long)(v)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline u32 apic_read(unsigned long reg)
{
 return ({ unsigned long __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_apic_ops.apic_read); if (sizeof(unsigned long) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_apic_ops.apic_read) / sizeof(void *))), [paravirt_opptr] "m" (pv_apic_ops.apic_read), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(reg)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_apic_ops.apic_read) / sizeof(void *))), [paravirt_opptr] "m" (pv_apic_ops.apic_read), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(reg)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (unsigned long)__eax; } __ret; });
}

static inline void setup_boot_clock(void)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_apic_ops.setup_boot_clock); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_apic_ops.setup_boot_clock) / sizeof(void *))), [paravirt_opptr] "m" (pv_apic_ops.setup_boot_clock), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void setup_secondary_clock(void)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_apic_ops.setup_secondary_clock); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_apic_ops.setup_secondary_clock) / sizeof(void *))), [paravirt_opptr] "m" (pv_apic_ops.setup_secondary_clock), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}


static inline void paravirt_post_allocator_init(void)
{
 if (pv_init_ops.post_allocator_init)
  (*pv_init_ops.post_allocator_init)();
}

static inline void paravirt_pagetable_setup_start(pgd_t *base)
{
 (*pv_mmu_ops.pagetable_setup_start)(base);
}

static inline void paravirt_pagetable_setup_done(pgd_t *base)
{
 (*pv_mmu_ops.pagetable_setup_done)(base);
}


static inline void startup_ipi_hook(int phys_apicid, unsigned long start_eip,
        unsigned long start_esp)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_apic_ops.startup_ipi_hook); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_apic_ops.startup_ipi_hook) / sizeof(void *))), [paravirt_opptr] "m" (pv_apic_ops.startup_ipi_hook), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(phys_apicid)), "1"((unsigned long)(start_eip)), "2"((unsigned long)(start_esp)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); })
                                        ;
}


static inline void paravirt_activate_mm(struct mm_struct *prev,
     struct mm_struct *next)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.activate_mm); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.activate_mm) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.activate_mm), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(prev)), "1" ((unsigned long)(next)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void arch_dup_mmap(struct mm_struct *oldmm,
     struct mm_struct *mm)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.dup_mmap); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.dup_mmap) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.dup_mmap), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(oldmm)), "1" ((unsigned long)(mm)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void arch_exit_mmap(struct mm_struct *mm)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.exit_mmap); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.exit_mmap) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.exit_mmap), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(mm)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void __flush_tlb(void)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.flush_tlb_user); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.flush_tlb_user) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.flush_tlb_user), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}
static inline void __flush_tlb_global(void)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.flush_tlb_kernel); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.flush_tlb_kernel) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.flush_tlb_kernel), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}
static inline void __flush_tlb_single(unsigned long addr)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.flush_tlb_single); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.flush_tlb_single) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.flush_tlb_single), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(addr)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void flush_tlb_others(cpumask_t cpumask, struct mm_struct *mm,
        unsigned long va)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.flush_tlb_others); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.flush_tlb_others) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.flush_tlb_others), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(&cpumask)), "1"((unsigned long)(mm)), "2"((unsigned long)(va)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline int paravirt_pgd_alloc(struct mm_struct *mm)
{
 return ({ int __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.pgd_alloc); if (sizeof(int) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pgd_alloc) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pgd_alloc), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(mm)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (int)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pgd_alloc) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pgd_alloc), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(mm)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (int)__eax; } __ret; });
}

static inline void paravirt_pgd_free(struct mm_struct *mm, pgd_t *pgd)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.pgd_free); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pgd_free) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pgd_free), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(mm)), "1" ((unsigned long)(pgd)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void paravirt_alloc_pte(struct mm_struct *mm, unsigned pfn)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.alloc_pte); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.alloc_pte) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.alloc_pte), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(mm)), "1" ((unsigned long)(pfn)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}
static inline void paravirt_release_pte(unsigned pfn)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.release_pte); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.release_pte) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.release_pte), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pfn)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void paravirt_alloc_pmd(struct mm_struct *mm, unsigned pfn)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.alloc_pmd); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.alloc_pmd) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.alloc_pmd), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(mm)), "1" ((unsigned long)(pfn)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void paravirt_alloc_pmd_clone(unsigned pfn, unsigned clonepfn,
         unsigned start, unsigned count)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.alloc_pmd_clone); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.alloc_pmd_clone) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.alloc_pmd_clone), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pfn)), "1"((unsigned long)(clonepfn)), "2"((unsigned long)(start)), "3"((unsigned long)(count)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}
static inline void paravirt_release_pmd(unsigned pfn)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.release_pmd); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.release_pmd) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.release_pmd), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pfn)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void paravirt_alloc_pud(struct mm_struct *mm, unsigned pfn)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.alloc_pud); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.alloc_pud) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.alloc_pud), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(mm)), "1" ((unsigned long)(pfn)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}
static inline void paravirt_release_pud(unsigned pfn)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.release_pud); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.release_pud) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.release_pud), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pfn)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}
static inline void pte_update(struct mm_struct *mm, unsigned long addr,
         pte_t *ptep)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.pte_update); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pte_update) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pte_update), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(mm)), "1"((unsigned long)(addr)), "2"((unsigned long)(ptep)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void pte_update_defer(struct mm_struct *mm, unsigned long addr,
        pte_t *ptep)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.pte_update_defer); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pte_update_defer) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pte_update_defer), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(mm)), "1"((unsigned long)(addr)), "2"((unsigned long)(ptep)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline pte_t __pte(pteval_t val)
{
 pteval_t ret;

 if (sizeof(pteval_t) > sizeof(long))
  ret = ({ pteval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.make_pte); if (sizeof(pteval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pte) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pte), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)), "1" ((unsigned long)((u64)val >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pteval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pte) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pte), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)), "1" ((unsigned long)((u64)val >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pteval_t)__eax; } __ret; })

                         ;
 else
  ret = ({ pteval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.make_pte); if (sizeof(pteval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pte) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pte), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pteval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pte) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pte), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pteval_t)__eax; } __ret; })

         ;


        return native_make_pte(ret);
}

static inline pteval_t pte_val(pte_t pte)
{
 pteval_t ret;

 if (sizeof(pteval_t) > sizeof(long))
  ret = ({ pteval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.pte_val); if (sizeof(pteval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pte_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pte_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pte.pte)), "1" ((unsigned long)((u64)pte.pte >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pteval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pte_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pte_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pte.pte)), "1" ((unsigned long)((u64)pte.pte >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pteval_t)__eax; } __ret; })
                                 ;
 else
  ret = ({ pteval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.pte_val); if (sizeof(pteval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pte_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pte_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pte.pte)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pteval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pte_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pte_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pte.pte)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pteval_t)__eax; } __ret; })
             ;

 return ret;
}

static inline pteval_t pte_flags(pte_t pte)
{
 pteval_t ret;

 if (sizeof(pteval_t) > sizeof(long))
  ret = ({ pteval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.pte_flags); if (sizeof(pteval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pte_flags) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pte_flags), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pte.pte)), "1" ((unsigned long)((u64)pte.pte >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pteval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pte_flags) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pte_flags), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pte.pte)), "1" ((unsigned long)((u64)pte.pte >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pteval_t)__eax; } __ret; })
                                 ;
 else
  ret = ({ pteval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.pte_flags); if (sizeof(pteval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pte_flags) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pte_flags), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pte.pte)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pteval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pte_flags) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pte_flags), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pte.pte)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pteval_t)__eax; } __ret; })
             ;




 return ret;
}

static inline pgd_t __pgd(pgdval_t val)
{
 pgdval_t ret;

 if (sizeof(pgdval_t) > sizeof(long))
  ret = ({ pgdval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.make_pgd); if (sizeof(pgdval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pgd) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pgd), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)), "1" ((unsigned long)((u64)val >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pgdval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pgd) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pgd), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)), "1" ((unsigned long)((u64)val >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pgdval_t)__eax; } __ret; })
                         ;
 else
  ret = ({ pgdval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.make_pgd); if (sizeof(pgdval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pgd) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pgd), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pgdval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pgd) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pgd), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pgdval_t)__eax; } __ret; })
         ;

 return (pgd_t) { ret };
}

static inline pgdval_t pgd_val(pgd_t pgd)
{
 pgdval_t ret;

 if (sizeof(pgdval_t) > sizeof(long))
  ret = ({ pgdval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.pgd_val); if (sizeof(pgdval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pgd_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pgd_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pgd.pgd)), "1" ((unsigned long)((u64)pgd.pgd >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pgdval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pgd_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pgd_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pgd.pgd)), "1" ((unsigned long)((u64)pgd.pgd >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pgdval_t)__eax; } __ret; })
                                  ;
 else
  ret = ({ pgdval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.pgd_val); if (sizeof(pgdval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pgd_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pgd_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pgd.pgd)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pgdval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pgd_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pgd_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pgd.pgd)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pgdval_t)__eax; } __ret; })
              ;

 return ret;
}


static inline pte_t ptep_modify_prot_start(struct mm_struct *mm, unsigned long addr,
        pte_t *ptep)
{
 pteval_t ret;

 ret = ({ pteval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.ptep_modify_prot_start); if (sizeof(pteval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.ptep_modify_prot_start) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.ptep_modify_prot_start), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(mm)), "1"((unsigned long)(addr)), "2"((unsigned long)(ptep)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pteval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.ptep_modify_prot_start) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.ptep_modify_prot_start), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(mm)), "1"((unsigned long)(addr)), "2"((unsigned long)(ptep)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pteval_t)__eax; } __ret; })
                   ;


        return native_make_pte(ret);
}

static inline void ptep_modify_prot_commit(struct mm_struct *mm, unsigned long addr,
        pte_t *ptep, pte_t pte)
{
 if (sizeof(pteval_t) > sizeof(long))

  pv_mmu_ops.ptep_modify_prot_commit(mm, addr, ptep, pte);
 else
  ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.ptep_modify_prot_commit); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.ptep_modify_prot_commit) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.ptep_modify_prot_commit), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(mm)), "1"((unsigned long)(addr)), "2"((unsigned long)(ptep)), "3"((unsigned long)(pte.pte)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); })
                               ;
}

static inline void set_pte(pte_t *ptep, pte_t pte)
{
 if (sizeof(pteval_t) > sizeof(long))
  ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.set_pte); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.set_pte) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.set_pte), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(ptep)), "1"((unsigned long)(pte.pte)), "2"((unsigned long)((u64)pte.pte >> 32)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); })
                                   ;
 else
  ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.set_pte); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.set_pte) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.set_pte), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(ptep)), "1" ((unsigned long)(pte.pte)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); })
               ;
}

static inline void set_pte_at(struct mm_struct *mm, unsigned long addr,
         pte_t *ptep, pte_t pte)
{
 if (sizeof(pteval_t) > sizeof(long))

  pv_mmu_ops.set_pte_at(mm, addr, ptep, pte);
 else
  ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.set_pte_at); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.set_pte_at) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.set_pte_at), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(mm)), "1"((unsigned long)(addr)), "2"((unsigned long)(ptep)), "3"((unsigned long)(pte.pte)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void set_pmd(pmd_t *pmdp, pmd_t pmd)
{
 pmdval_t val = native_pmd_val(pmd);

 if (sizeof(pmdval_t) > sizeof(long))
  ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.set_pmd); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.set_pmd) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.set_pmd), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pmdp)), "1"((unsigned long)(val)), "2"((unsigned long)((u64)val >> 32)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
 else
  ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.set_pmd); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.set_pmd) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.set_pmd), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pmdp)), "1" ((unsigned long)(val)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}


static inline pmd_t __pmd(pmdval_t val)
{
 pmdval_t ret;

 if (sizeof(pmdval_t) > sizeof(long))
  ret = ({ pmdval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.make_pmd); if (sizeof(pmdval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pmd) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pmd), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)), "1" ((unsigned long)((u64)val >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pmdval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pmd) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pmd), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)), "1" ((unsigned long)((u64)val >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pmdval_t)__eax; } __ret; })
                         ;
 else
  ret = ({ pmdval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.make_pmd); if (sizeof(pmdval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pmd) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pmd), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pmdval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pmd) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pmd), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pmdval_t)__eax; } __ret; })
         ;

 return (pmd_t) { ret };
}

static inline pmdval_t pmd_val(pmd_t pmd)
{
 pmdval_t ret;

 if (sizeof(pmdval_t) > sizeof(long))
  ret = ({ pmdval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.pmd_val); if (sizeof(pmdval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pmd_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pmd_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pmd.pmd)), "1" ((unsigned long)((u64)pmd.pmd >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pmdval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pmd_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pmd_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pmd.pmd)), "1" ((unsigned long)((u64)pmd.pmd >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pmdval_t)__eax; } __ret; })
                                  ;
 else
  ret = ({ pmdval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.pmd_val); if (sizeof(pmdval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pmd_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pmd_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pmd.pmd)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pmdval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pmd_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pmd_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pmd.pmd)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pmdval_t)__eax; } __ret; })
              ;

 return ret;
}

static inline void set_pud(pud_t *pudp, pud_t pud)
{
 pudval_t val = native_pud_val(pud);

 if (sizeof(pudval_t) > sizeof(long))
  ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.set_pud); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.set_pud) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.set_pud), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pudp)), "1"((unsigned long)(val)), "2"((unsigned long)((u64)val >> 32)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); })
                           ;
 else
  ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.set_pud); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.set_pud) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.set_pud), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pudp)), "1" ((unsigned long)(val)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); })
           ;
}

static inline pud_t __pud(pudval_t val)
{
 pudval_t ret;

 if (sizeof(pudval_t) > sizeof(long))
  ret = ({ pudval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.make_pud); if (sizeof(pudval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pud) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pud), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)), "1" ((unsigned long)((u64)val >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pudval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pud) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pud), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)), "1" ((unsigned long)((u64)val >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pudval_t)__eax; } __ret; })
                         ;
 else
  ret = ({ pudval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.make_pud); if (sizeof(pudval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pud) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pud), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pudval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.make_pud) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.make_pud), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(val)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pudval_t)__eax; } __ret; })
         ;

 return (pud_t) { ret };
}

static inline pudval_t pud_val(pud_t pud)
{
 pudval_t ret;

 if (sizeof(pudval_t) > sizeof(long))
  ret = ({ pudval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.pud_val); if (sizeof(pudval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pud_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pud_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pud.pud)), "1" ((unsigned long)((u64)pud.pud >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pudval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pud_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pud_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pud.pud)), "1" ((unsigned long)((u64)pud.pud >> 32)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pudval_t)__eax; } __ret; })
                                  ;
 else
  ret = ({ pudval_t __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_mmu_ops.pud_val); if (sizeof(pudval_t) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pud_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pud_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pud.pud)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pudval_t)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.pud_val) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.pud_val), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pud.pud)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (pudval_t)__eax; } __ret; })
              ;

 return ret;
}

static inline void set_pgd(pgd_t *pgdp, pgd_t pgd)
{
 pgdval_t val = native_pgd_val(pgd);

 if (sizeof(pgdval_t) > sizeof(long))
  ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.set_pgd); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.set_pgd) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.set_pgd), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pgdp)), "1"((unsigned long)(val)), "2"((unsigned long)((u64)val >> 32)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); })
                           ;
 else
  ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.set_pgd); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.set_pgd) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.set_pgd), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(pgdp)), "1" ((unsigned long)(val)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); })
           ;
}

static inline void pgd_clear(pgd_t *pgdp)
{
 set_pgd(pgdp, __pgd(0));
}

static inline void pud_clear(pud_t *pudp)
{
 set_pud(pudp, __pud(0));
}
static inline void set_pte_atomic(pte_t *ptep, pte_t pte)
{
 set_pte(ptep, pte);
}

static inline void set_pte_present(struct mm_struct *mm, unsigned long addr,
       pte_t *ptep, pte_t pte)
{
 set_pte(ptep, pte);
}

static inline void pte_clear(struct mm_struct *mm, unsigned long addr,
        pte_t *ptep)
{
 set_pte_at(mm, addr, ptep, __pte(0));
}

static inline void pmd_clear(pmd_t *pmdp)
{
 set_pmd(pmdp, __pmd(0));
}



enum paravirt_lazy_mode {
 PARAVIRT_LAZY_NONE,
 PARAVIRT_LAZY_MMU,
 PARAVIRT_LAZY_CPU,
};

enum paravirt_lazy_mode paravirt_get_lazy_mode(void);
void paravirt_enter_lazy_cpu(void);
void paravirt_leave_lazy_cpu(void);
void paravirt_enter_lazy_mmu(void);
void paravirt_leave_lazy_mmu(void);
void paravirt_leave_lazy(enum paravirt_lazy_mode mode);


static inline void arch_enter_lazy_cpu_mode(void)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.lazy_mode.enter); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.lazy_mode.enter) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.lazy_mode.enter), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void arch_leave_lazy_cpu_mode(void)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_cpu_ops.lazy_mode.leave); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_cpu_ops.lazy_mode.leave) / sizeof(void *))), [paravirt_opptr] "m" (pv_cpu_ops.lazy_mode.leave), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void arch_flush_lazy_cpu_mode(void)
{
 if (__builtin_expect(!!(paravirt_get_lazy_mode() == PARAVIRT_LAZY_CPU), 0)) {
  arch_leave_lazy_cpu_mode();
  arch_enter_lazy_cpu_mode();
 }
}



static inline void arch_enter_lazy_mmu_mode(void)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.lazy_mode.enter); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.lazy_mode.enter) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.lazy_mode.enter), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void arch_leave_lazy_mmu_mode(void)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_mmu_ops.lazy_mode.leave); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_mmu_ops.lazy_mode.leave) / sizeof(void *))), [paravirt_opptr] "m" (pv_mmu_ops.lazy_mode.leave), [paravirt_clobber] "i" (((1 << 9) - 1)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline void arch_flush_lazy_mmu_mode(void)
{
 if (__builtin_expect(!!(paravirt_get_lazy_mode() == PARAVIRT_LAZY_MMU), 0)) {
  arch_leave_lazy_mmu_mode();
  arch_enter_lazy_mmu_mode();
 }
}

static inline void __set_fixmap(unsigned idx,
    unsigned long phys, pgprot_t flags)
{
 pv_mmu_ops.set_fixmap(idx, phys, flags);
}

void _paravirt_nop(void);


void paravirt_use_bytelocks(void);



static inline int __raw_spin_is_locked(struct raw_spinlock *lock)
{
 return ({ int __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_lock_ops.spin_is_locked); if (sizeof(int) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_lock_ops.spin_is_locked) / sizeof(void *))), [paravirt_opptr] "m" (pv_lock_ops.spin_is_locked), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(lock)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (int)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_lock_ops.spin_is_locked) / sizeof(void *))), [paravirt_opptr] "m" (pv_lock_ops.spin_is_locked), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(lock)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (int)__eax; } __ret; });
}

static inline int __raw_spin_is_contended(struct raw_spinlock *lock)
{
 return ({ int __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_lock_ops.spin_is_contended); if (sizeof(int) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_lock_ops.spin_is_contended) / sizeof(void *))), [paravirt_opptr] "m" (pv_lock_ops.spin_is_contended), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(lock)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (int)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_lock_ops.spin_is_contended) / sizeof(void *))), [paravirt_opptr] "m" (pv_lock_ops.spin_is_contended), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(lock)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (int)__eax; } __ret; });
}

static inline __attribute__((always_inline)) void __raw_spin_lock(struct raw_spinlock *lock)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_lock_ops.spin_lock); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_lock_ops.spin_lock) / sizeof(void *))), [paravirt_opptr] "m" (pv_lock_ops.spin_lock), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(lock)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}

static inline __attribute__((always_inline)) int __raw_spin_trylock(struct raw_spinlock *lock)
{
 return ({ int __ret; unsigned long __edi, __esi, __edx, __ecx, __eax; ((void)pv_lock_ops.spin_trylock); if (sizeof(int) > sizeof(unsigned long)) { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_lock_ops.spin_trylock) / sizeof(void *))), [paravirt_opptr] "m" (pv_lock_ops.spin_trylock), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(lock)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (int)((((u64)__edx) << 32) | __eax); } else { asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx), "=a" (__eax) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_lock_ops.spin_trylock) / sizeof(void *))), [paravirt_opptr] "m" (pv_lock_ops.spin_trylock), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(lock)) : "memory", "cc" , "r8", "r9", "r10", "r11"); __ret = (int)__eax; } __ret; });
}

static inline __attribute__((always_inline)) void __raw_spin_unlock(struct raw_spinlock *lock)
{
 ({ unsigned long __edi, __esi, __edx, __ecx; ((void)pv_lock_ops.spin_unlock); asm ("" "771:\n\t" "call *%[paravirt_opptr];" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n" "" : "=D" (__edi), "=S" (__esi), "=d" (__edx), "=c" (__ecx) : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_lock_ops.spin_unlock) / sizeof(void *))), [paravirt_opptr] "m" (pv_lock_ops.spin_unlock), [paravirt_clobber] "i" (((1 << 9) - 1)), "0" ((unsigned long)(lock)) : "memory", "cc" , "rax", "r8", "r9", "r10", "r11"); });
}




struct paravirt_patch_site {
 u8 *instr;
 u8 instrtype;
 u8 len;
 u16 clobbers;
};

extern struct paravirt_patch_site __parainstructions[],
 __parainstructions_end[];
static inline unsigned long __raw_local_save_flags(void)
{
 unsigned long f;

 asm ("771:\n\t" "pushq %%rdi;" "call *%[paravirt_opptr];" "popq %%rdi;" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n"


       : "=a"(f)
       : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_irq_ops.save_fl) / sizeof(void *))), [paravirt_opptr] "m" (pv_irq_ops.save_fl),
         [paravirt_clobber] "i" ((1 << 0))
       : "memory", "cc" , "r8", "r9", "r10", "r11", "rdi", "rcx" , "rdx", "rsi");
 return f;
}

static inline void raw_local_irq_restore(unsigned long f)
{
 asm ("771:\n\t" "pushq %%rdi;" "call *%[paravirt_opptr];" "popq %%rdi;" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n"


       : "=a"(f)
       : "D"(f),
         [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_irq_ops.restore_fl) / sizeof(void *))), [paravirt_opptr] "m" (pv_irq_ops.restore_fl),
         [paravirt_clobber] "i" ((1 << 0))
       : "memory", "cc" , "r8", "r9", "r10", "r11", "rcx" , "rdx", "rsi");
}

static inline void raw_local_irq_disable(void)
{
 asm ("771:\n\t" "pushq %%rdi;" "call *%[paravirt_opptr];" "popq %%rdi;" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n"


       :
       : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_irq_ops.irq_disable) / sizeof(void *))), [paravirt_opptr] "m" (pv_irq_ops.irq_disable),
         [paravirt_clobber] "i" ((1 << 0))
       : "memory", "eax", "cc" , "r8", "r9", "r10", "r11", "rcx" , "rdx", "rsi");
}

static inline void raw_local_irq_enable(void)
{
 asm ("771:\n\t" "pushq %%rdi;" "call *%[paravirt_opptr];" "popq %%rdi;" "\n" "772:\n" ".pushsection .parainstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " " 771b\n" "  .byte " "%c[paravirt_typenum]" "\n" "  .byte 772b-771b\n" "  .short " "%c[paravirt_clobber]" "\n" ".popsection\n"


       :
       : [paravirt_typenum] "i" ((((size_t) &((struct paravirt_patch_template *)0)->pv_irq_ops.irq_enable) / sizeof(void *))), [paravirt_opptr] "m" (pv_irq_ops.irq_enable),
         [paravirt_clobber] "i" ((1 << 0))
       : "memory", "eax", "cc" , "r8", "r9", "r10", "r11", "rcx" , "rdx", "rsi");
}

static inline unsigned long __raw_local_irq_save(void)
{
 unsigned long f;

 f = __raw_local_save_flags();
 raw_local_irq_disable();
 return f;
}
static __inline__ __attribute__((__const__)) int get_order(unsigned long size)
{
 int order;

 size = (size - 1) >> (12 - 1);
 order = -1;
 do {
  size >>= 1;
  order++;
 } while (size);
 return order;
}
struct task_struct;
struct exec_domain;





struct task_struct;
struct mm_struct;

struct vm86_regs {



 long ebx;
 long ecx;
 long edx;
 long esi;
 long edi;
 long ebp;
 long eax;
 long __null_ds;
 long __null_es;
 long __null_fs;
 long __null_gs;
 long orig_eax;
 long eip;
 unsigned short cs, __csh;
 long eflags;
 long esp;
 unsigned short ss, __ssh;



 unsigned short es, __esh;
 unsigned short ds, __dsh;
 unsigned short fs, __fsh;
 unsigned short gs, __gsh;
};

struct revectored_struct {
 unsigned long __map[8];
};

struct vm86_struct {
 struct vm86_regs regs;
 unsigned long flags;
 unsigned long screen_bitmap;
 unsigned long cpu_type;
 struct revectored_struct int_revectored;
 struct revectored_struct int21_revectored;
};






struct vm86plus_info_struct {
 unsigned long force_return_for_pic:1;
 unsigned long vm86dbg_active:1;
 unsigned long vm86dbg_TFpendig:1;
 unsigned long unused:28;
 unsigned long is_vm86pus:1;
 unsigned char vm86dbg_intxxtab[32];
};
struct vm86plus_struct {
 struct vm86_regs regs;
 unsigned long flags;
 unsigned long screen_bitmap;
 unsigned long cpu_type;
 struct revectored_struct int_revectored;
 struct revectored_struct int21_revectored;
 struct vm86plus_info_struct vm86plus;
};







struct ptrace_bts_config {

 __u32 size;

 __u32 flags;

 __u32 signal;

 __u32 bts_size;
};



typedef int (*initcall_t)(void);
typedef void (*exitcall_t)(void);

extern initcall_t __con_initcall_start[], __con_initcall_end[];
extern initcall_t __security_initcall_start[], __security_initcall_end[];


extern int do_one_initcall(initcall_t fn);
extern char __attribute__ ((__section__(".init.data"))) boot_command_line[];
extern char *saved_command_line;
extern unsigned int reset_devices;


void setup_arch(char **);
void prepare_namespace(void);

extern void (*late_time_init)(void);
struct obs_kernel_param {
 char *str;
 int (*setup_func)(char *);
 int early;
};
void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) parse_early_param(void);

struct cpuinfo_x86;







enum bts_qualifier {
 BTS_INVALID = 0,
 BTS_BRANCH,
 BTS_TASK_ARRIVES,
 BTS_TASK_DEPARTS
};

struct bts_struct {
 u64 qualifier;
 union {

  struct {
   u64 from_ip;
   u64 to_ip;
  } lbr;


  u64 jiffies;
 } variant;
};





extern int ds_allocate(void **, size_t);
extern int ds_free(void **);
extern int ds_get_bts_size(void *);
extern int ds_get_bts_end(void *);
extern int ds_get_bts_index(void *);
extern int ds_set_overflow(void *, int);
extern int ds_get_overflow(void *);
extern int ds_clear(void *);
extern int ds_read_bts(void *, int, struct bts_struct *);
extern int ds_write_bts(void *, struct bts_struct *);
extern unsigned long ds_debugctl_mask(void);
extern void __attribute__ ((__section__(".cpuinit.text"))) __attribute__((__cold__)) ds_init_intel(struct cpuinfo_x86 *c);
extern char early_idt_handlers[32][10];
struct pt_regs {
 unsigned long r15;
 unsigned long r14;
 unsigned long r13;
 unsigned long r12;
 unsigned long bp;
 unsigned long bx;

 unsigned long r11;
 unsigned long r10;
 unsigned long r9;
 unsigned long r8;
 unsigned long ax;
 unsigned long cx;
 unsigned long dx;
 unsigned long si;
 unsigned long di;
 unsigned long orig_ax;


 unsigned long ip;
 unsigned long cs;
 unsigned long flags;
 unsigned long sp;
 unsigned long ss;

};
struct task_struct;

extern void ptrace_bts_take_timestamp(struct task_struct *, enum bts_qualifier);

extern unsigned long profile_pc(struct pt_regs *regs);

extern unsigned long
convert_ip_to_linear(struct task_struct *child, struct pt_regs *regs);





void signal_fault(struct pt_regs *regs, void *frame, char *where);


static inline unsigned long regs_return_value(struct pt_regs *regs)
{
 return regs->ax;
}
static inline int user_mode(struct pt_regs *regs)
{



 return !!(regs->cs & 3);

}

static inline int user_mode_vm(struct pt_regs *regs)
{




 return user_mode(regs);

}

static inline int v8086_mode(struct pt_regs *regs)
{



 return 0;

}







static inline unsigned long kernel_trap_sp(struct pt_regs *regs)
{



 return regs->sp;

}

static inline unsigned long instruction_pointer(struct pt_regs *regs)
{
 return regs->ip;
}

static inline unsigned long frame_pointer(struct pt_regs *regs)
{
 return regs->bp;
}





extern void user_enable_single_step(struct task_struct *);
extern void user_disable_single_step(struct task_struct *);

extern void user_enable_block_step(struct task_struct *);






struct user_desc;
extern int do_get_thread_area(struct task_struct *p, int idx,
         struct user_desc *info);
extern int do_set_thread_area(struct task_struct *p, int idx,
         struct user_desc *info, int can_allocate);
struct kernel_vm86_regs {



 struct pt_regs pt;



 unsigned short es, __esh;
 unsigned short ds, __dsh;
 unsigned short fs, __fsh;
 unsigned short gs, __gsh;
};

struct kernel_vm86_struct {
 struct kernel_vm86_regs regs;
 unsigned long flags;
 unsigned long screen_bitmap;
 unsigned long cpu_type;
 struct revectored_struct int_revectored;
 struct revectored_struct int21_revectored;
 struct vm86plus_info_struct vm86plus;
 struct pt_regs *regs32;
};
static inline int handle_vm86_trap(struct kernel_vm86_regs *a, long b, int c)
{
 return 0;
}







struct info {
 long ___orig_eip;
 long ___ebx;
 long ___ecx;
 long ___edx;
 long ___esi;
 long ___edi;
 long ___ebp;
 long ___eax;
 long ___ds;
 long ___es;
 long ___fs;
 long ___orig_eax;
 long ___eip;
 long ___cs;
 long ___eflags;
 long ___esp;
 long ___ss;
 long ___vm86_es;
 long ___vm86_ds;
 long ___vm86_fs;
 long ___vm86_gs;
};


struct _fpstate {
 __u16 cwd;
 __u16 swd;
 __u16 twd;

 __u16 fop;
 __u64 rip;
 __u64 rdp;
 __u32 mxcsr;
 __u32 mxcsr_mask;
 __u32 st_space[32];
 __u32 xmm_space[64];
 __u32 reserved2[24];
};


struct sigcontext {
 unsigned long r8;
 unsigned long r9;
 unsigned long r10;
 unsigned long r11;
 unsigned long r12;
 unsigned long r13;
 unsigned long r14;
 unsigned long r15;
 unsigned long di;
 unsigned long si;
 unsigned long bp;
 unsigned long bx;
 unsigned long dx;
 unsigned long ax;
 unsigned long cx;
 unsigned long sp;
 unsigned long ip;
 unsigned long flags;
 unsigned short cs;
 unsigned short gs;
 unsigned short fs;
 unsigned short __pad0;
 unsigned long err;
 unsigned long trapno;
 unsigned long oldmask;
 unsigned long cr2;
 struct _fpstate *fpstate;
 unsigned long reserved1[8];
};




extern struct task_struct *get_current(void);










static inline void set_64bit( unsigned long *ptr, unsigned long val)
{
 *ptr = val;
}
static inline unsigned long __xchg(unsigned long x, void *ptr,
       int size)
{
 switch (size) {
 case 1:
  asm ("xchgb %b0,%1"
        : "=q" (x)
        : "m" (*(( long *)(ptr))), "0" (x)
        : "memory");
  break;
 case 2:
  asm ("xchgw %w0,%1"
        : "=r" (x)
        : "m" (*(( long *)(ptr))), "0" (x)
        : "memory");
  break;
 case 4:
  asm ("xchgl %k0,%1"
        : "=r" (x)
        : "m" (*(( long *)(ptr))), "0" (x)
        : "memory");
  break;
 case 8:
  asm ("xchgq %0,%1"
        : "=r" (x)
        : "m" (*(( long *)(ptr))), "0" (x)
        : "memory");
  break;
 }
 return x;
}
static inline unsigned long __cmpxchg( void *ptr, unsigned long old,
          unsigned long _new, int size)
{
 unsigned long prev;
 switch (size) {
 case 1:
  asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "cmpxchgb %b1,%2"
        : "=a"(prev)
        : "q"(_new), "m"(*(( long *)(ptr))), "0"(old)
        : "memory");
  return prev;
 case 2:
  asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "cmpxchgw %w1,%2"
        : "=a"(prev)
        : "r"(_new), "m"(*(( long *)(ptr))), "0"(old)
        : "memory");
  return prev;
 case 4:
  asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "cmpxchgl %k1,%2"
        : "=a"(prev)
        : "r"(_new), "m"(*(( long *)(ptr))), "0"(old)
        : "memory");
  return prev;
 case 8:
  asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "cmpxchgq %1,%2"
        : "=a"(prev)
        : "r"(_new), "m"(*(( long *)(ptr))), "0"(old)
        : "memory");
  return prev;
 }
 return old;
}






static inline unsigned long __sync_cmpxchg( void *ptr,
        unsigned long old,
        unsigned long _new, int size)
{
 unsigned long prev;
 switch (size) {
 case 1:
  asm ("lock; cmpxchgb %b1,%2"
        : "=a"(prev)
        : "q"(_new), "m"(*(( long *)(ptr))), "0"(old)
        : "memory");
  return prev;
 case 2:
  asm ("lock; cmpxchgw %w1,%2"
        : "=a"(prev)
        : "r"(_new), "m"(*(( long *)(ptr))), "0"(old)
        : "memory");
  return prev;
 case 4:
  asm ("lock; cmpxchgl %1,%2"
        : "=a"(prev)
        : "r"(_new), "m"(*(( long *)(ptr))), "0"(old)
        : "memory");
  return prev;
 }
 return old;
}

static inline unsigned long __cmpxchg_local( void *ptr,
         unsigned long old,
         unsigned long _new, int size)
{
 unsigned long prev;
 switch (size) {
 case 1:
  asm ("cmpxchgb %b1,%2"
        : "=a"(prev)
        : "q"(_new), "m"(*(( long *)(ptr))), "0"(old)
        : "memory");
  return prev;
 case 2:
  asm ("cmpxchgw %w1,%2"
        : "=a"(prev)
        : "r"(_new), "m"(*(( long *)(ptr))), "0"(old)
        : "memory");
  return prev;
 case 4:
  asm ("cmpxchgl %k1,%2"
        : "=a"(prev)
        : "r"(_new), "m"(*(( long *)(ptr))), "0"(old)
        : "memory");
  return prev;
 case 8:
  asm ("cmpxchgq %1,%2"
        : "=a"(prev)
        : "r"(_new), "m"(*(( long *)(ptr))), "0"(old)
        : "memory");
  return prev;
 }
 return old;
}


  extern void trace_softirqs_on(unsigned long ip);
  extern void trace_softirqs_off(unsigned long ip);
  extern void trace_hardirqs_on(void);
  extern void trace_hardirqs_off(void);
 extern void stop_critical_timings(void);
 extern void start_critical_timings(void);







static inline unsigned long native_save_fl(void)
{
 unsigned long flags;







 asm ("# __raw_save_flags\n\t"
       "pushf ; pop %0"
       : "=r" (flags)
       :
       : "memory");

 return flags;
}

static inline void native_restore_fl(unsigned long flags)
{
 asm ("push %0 ; popf"
       :
       :"g" (flags)
       :"memory", "cc");
}

static inline void native_irq_disable(void)
{
 asm ("cli": : :"memory");
}

static inline void native_irq_enable(void)
{
 asm ("sti": : :"memory");
}

static inline void native_safe_halt(void)
{
 asm ("sti; hlt": : :"memory");
}

static inline void native_halt(void)
{
 asm ("hlt": : :"memory");
}
static inline int raw_irqs_disabled_flags(unsigned long flags)
{
 return !(flags & 0x00000200);
}

static inline int raw_irqs_disabled(void)
{
 unsigned long flags = __raw_local_save_flags();

 return raw_irqs_disabled_flags(flags);
}







static inline void trace_hardirqs_fixup_flags(unsigned long flags)
{
 if (raw_irqs_disabled_flags(flags))
  trace_hardirqs_off();
 else
  trace_hardirqs_on();
}

static inline void trace_hardirqs_fixup(void)
{
 unsigned long flags = __raw_local_save_flags();

 trace_hardirqs_fixup_flags(flags);
}
extern void native_load_gs_index(unsigned);
static inline unsigned long get_limit(unsigned long segment)
{
 unsigned long __limit;
 asm("lsll %1,%0" : "=r" (__limit) : "r" (segment));
 return __limit + 1;
}

static inline void native_clts(void)
{
 asm ("clts");
}
static unsigned long __force_order;

static inline unsigned long native_read_cr0(void)
{
 unsigned long val;
 asm ("mov %%cr0,%0\n\t" : "=r" (val), "=m" (__force_order));
 return val;
}

static inline void native_write_cr0(unsigned long val)
{
 asm ("mov %0,%%cr0": : "r" (val), "m" (__force_order));
}

static inline unsigned long native_read_cr2(void)
{
 unsigned long val;
 asm ("mov %%cr2,%0\n\t" : "=r" (val), "=m" (__force_order));
 return val;
}

static inline void native_write_cr2(unsigned long val)
{
 asm ("mov %0,%%cr2": : "r" (val), "m" (__force_order));
}

static inline unsigned long native_read_cr3(void)
{
 unsigned long val;
 asm ("mov %%cr3,%0\n\t" : "=r" (val), "=m" (__force_order));
 return val;
}

static inline void native_write_cr3(unsigned long val)
{
 asm ("mov %0,%%cr3": : "r" (val), "m" (__force_order));
}

static inline unsigned long native_read_cr4(void)
{
 unsigned long val;
 asm ("mov %%cr4,%0\n\t" : "=r" (val), "=m" (__force_order));
 return val;
}

static inline unsigned long native_read_cr4_safe(void)
{
 unsigned long val;
 val = native_read_cr4();

 return val;
}

static inline void native_write_cr4(unsigned long val)
{
 asm ("mov %0,%%cr4": : "r" (val), "m" (__force_order));
}


static inline unsigned long native_read_cr8(void)
{
 unsigned long cr8;
 asm ("movq %%cr8,%0" : "=r" (cr8));
 return cr8;
}

static inline void native_write_cr8(unsigned long val)
{
 asm ("movq %0,%%cr8" :: "r" (val) : "memory");
}


static inline void native_wbinvd(void)
{
 asm ("wbinvd": : :"memory");
}
static inline void clflush( void *__p)
{
 asm ("clflush %0" : "+m" (*( char *)__p));
}



void disable_hlt(void);
void enable_hlt(void);

void cpu_idle_wait(void);

extern unsigned long arch_align_stack(unsigned long sp);
extern void free_init_pages(char *what, unsigned long begin, unsigned long end);

void default_idle(void);
static inline void rdtsc_barrier(void)
{
 asm ("661:\n\t" ".byte 0x66,0x66,0x90\n" "\n662:\n" ".section .altinstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661b\n" " " ".quad" " " "663f\n" "	 .byte %c0\n" "	 .byte 662b-661b\n" "	 .byte 664f-663f\n" ".previous\n" ".section .altinstr_replacement,\"ax\"\n" "663:\n\t" "mfence" "\n664:\n" ".previous" :: "i" ((3*32+17)) : "memory");
 asm ("661:\n\t" ".byte 0x66,0x66,0x90\n" "\n662:\n" ".section .altinstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661b\n" " " ".quad" " " "663f\n" "	 .byte %c0\n" "	 .byte 662b-661b\n" "	 .byte 664f-663f\n" ".previous\n" ".section .altinstr_replacement,\"ax\"\n" "663:\n\t" "lfence" "\n664:\n" ".previous" :: "i" ((3*32+18)) : "memory");
}

struct x8664_pda {
 struct task_struct *pcurrent;
 unsigned long data_offset;

 unsigned long kernelstack;
 unsigned long oldrsp;
 int irqcount;
 unsigned int cpunumber;





 char *irqstackptr;
 short nodenumber;
 short in_bootmem;
 unsigned int __softirq_pending;
 unsigned int __nmi_count;
 short mmu_state;
 short isidle;
 struct mm_struct *active_mm;
 unsigned apic_timer_irqs;
 unsigned irq0_irqs;
 unsigned irq_resched_count;
 unsigned irq_call_count;
 unsigned irq_tlb_count;
 unsigned irq_thermal_count;
 unsigned irq_threshold_count;
 unsigned irq_spurious_count;
} __attribute__((__aligned__((1 << (7)))));

extern struct x8664_pda **_cpu_pda;
extern void pda_init(int);







extern void __bad_pda_field(void) __attribute__((noreturn));





extern struct x8664_pda _proxy_pda;







extern void setup_per_cpu_areas(void);

extern __typeof__(struct x8664_pda) per_cpu__pda;







static inline unsigned long long native_read_tscp(unsigned int *aux)
{
 unsigned long low, high;
 asm (".byte 0x0f,0x01,0xf9"
       : "=a" (low), "=d" (high), "=c" (*aux));
 return low | ((u64)high << 32);
}
static inline unsigned long long native_read_msr(unsigned int msr)
{
 unsigned low, high;

 asm ("rdmsr" : "=a" (low), "=d" (high) : "c" (msr));
 return ((low) | ((u64)(high) << 32));
}

static inline unsigned long long native_read_msr_safe(unsigned int msr,
            int *err)
{
 unsigned low, high;

 asm ("2: rdmsr ; xor %[err],%[err]\n"
       "1:\n\t"
       ".section .fixup,\"ax\"\n\t"
       "3:  mov %[fault],%[err] ; jmp 1b\n\t"
       ".previous\n\t"
       " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "2b" "," "3b" "\n" " .previous\n"
       : [err] "=r" (*err), "=a" (low), "=d" (high)
       : "c" (msr), [fault] "i" (-14));
 return ((low) | ((u64)(high) << 32));
}

static inline void native_write_msr(unsigned int msr,
        unsigned low, unsigned high)
{
 asm ("wrmsr" : : "c" (msr), "a"(low), "d" (high) : "memory");
}

static inline int native_write_msr_safe(unsigned int msr,
     unsigned low, unsigned high)
{
 int err;
 asm ("2: wrmsr ; xor %[err],%[err]\n"
       "1:\n\t"
       ".section .fixup,\"ax\"\n\t"
       "3:  mov %[fault],%[err] ; jmp 1b\n\t"
       ".previous\n\t"
       " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "2b" "," "3b" "\n" " .previous\n"
       : [err] "=a" (err)
       : "c" (msr), "0" (low), "d" (high),
         [fault] "i" (-14)
       : "memory");
 return err;
}

extern unsigned long long native_read_tsc(void);

static inline __attribute__((always_inline)) unsigned long long __native_read_tsc(void)
{
 unsigned low, high;

 rdtsc_barrier();
 asm ("rdtsc" : "=a" (low), "=d" (high));
 rdtsc_barrier();

 return ((low) | ((u64)(high) << 32));
}

static inline unsigned long long native_read_pmc(int counter)
{
 unsigned low, high;

 asm ("rdpmc" : "=a" (low), "=d" (high) : "c" (counter));
 return ((low) | ((u64)(high) << 32));
}
int rdmsr_on_cpu(unsigned int cpu, u32 msr_no, u32 *l, u32 *h);
int wrmsr_on_cpu(unsigned int cpu, u32 msr_no, u32 l, u32 h);
int rdmsr_safe_on_cpu(unsigned int cpu, u32 msr_no, u32 *l, u32 *h);
int wrmsr_safe_on_cpu(unsigned int cpu, u32 msr_no, u32 l, u32 h);



struct exec_domain;
struct pt_regs;

extern int register_exec_domain(struct exec_domain *);
extern int unregister_exec_domain(struct exec_domain *);
extern int __set_personality(unsigned long);
enum {
 ADDR_NO_RANDOMIZE = 0x0040000,
 FDPIC_FUNCPTRS = 0x0080000,


 MMAP_PAGE_ZERO = 0x0100000,
 ADDR_COMPAT_LAYOUT = 0x0200000,
 READ_IMPLIES_EXEC = 0x0400000,
 ADDR_LIMIT_32BIT = 0x0800000,
 SHORT_INODE = 0x1000000,
 WHOLE_SECONDS = 0x2000000,
 STICKY_TIMEOUTS = 0x4000000,
 ADDR_LIMIT_3GB = 0x8000000,
};
enum {
 PER_LINUX = 0x0000,
 PER_LINUX_32BIT = 0x0000 | ADDR_LIMIT_32BIT,
 PER_LINUX_FDPIC = 0x0000 | FDPIC_FUNCPTRS,
 PER_SVR4 = 0x0001 | STICKY_TIMEOUTS | MMAP_PAGE_ZERO,
 PER_SVR3 = 0x0002 | STICKY_TIMEOUTS | SHORT_INODE,
 PER_SCOSVR3 = 0x0003 | STICKY_TIMEOUTS |
      WHOLE_SECONDS | SHORT_INODE,
 PER_OSR5 = 0x0003 | STICKY_TIMEOUTS | WHOLE_SECONDS,
 PER_WYSEV386 = 0x0004 | STICKY_TIMEOUTS | SHORT_INODE,
 PER_ISCR4 = 0x0005 | STICKY_TIMEOUTS,
 PER_BSD = 0x0006,
 PER_SUNOS = 0x0006 | STICKY_TIMEOUTS,
 PER_XENIX = 0x0007 | STICKY_TIMEOUTS | SHORT_INODE,
 PER_LINUX32 = 0x0008,
 PER_LINUX32_3GB = 0x0008 | ADDR_LIMIT_3GB,
 PER_IRIX32 = 0x0009 | STICKY_TIMEOUTS,
 PER_IRIXN32 = 0x000a | STICKY_TIMEOUTS,
 PER_IRIX64 = 0x000b | STICKY_TIMEOUTS,
 PER_RISCOS = 0x000c,
 PER_SOLARIS = 0x000d | STICKY_TIMEOUTS,
 PER_UW7 = 0x000e | STICKY_TIMEOUTS | MMAP_PAGE_ZERO,
 PER_OSF4 = 0x000f,
 PER_HPUX = 0x0010,
 PER_MASK = 0x00ff,
};
typedef void (*handler_t)(int, struct pt_regs *);

struct exec_domain {
 char *name;
 handler_t handler;
 unsigned char pers_low;
 unsigned char pers_high;
 unsigned long *signal_map;
 unsigned long *signal_invmap;
 struct map_segment *err_map;
 struct map_segment *socktype_map;
 struct map_segment *sockopt_map;
 struct map_segment *af_map;
 struct module *module;
 struct exec_domain *next;
};
static inline void *current_text_addr(void)
{
 void *pc;

 asm ("mov $1f, %0; 1:":"=r" (pc));

 return pc;
}
struct cpuinfo_x86 {
 __u8 x86;
 __u8 x86_vendor;
 __u8 x86_model;
 __u8 x86_mask;
 int x86_tlbsize;
 __u8 x86_virt_bits;
 __u8 x86_phys_bits;

 __u8 x86_coreid_bits;

 __u32 extended_cpuid_level;


 int cpuid_level;
 __u32 x86_capability[8];
 char x86_vendor_id[16];
 char x86_model_id[64];

 int x86_cache_size;
 int x86_cache_alignment;
 int x86_power;
 unsigned long loops_per_jiffy;


 cpumask_t llc_shared_map;


 u16 x86_max_cores;
 u16 apicid;
 u16 initial_apicid;
 u16 x86_clflush_size;


 u16 booted_cores;

 u16 phys_proc_id;

 u16 cpu_core_id;

 u16 cpu_index;

} __attribute__((__aligned__((1 << (7)))));
extern struct cpuinfo_x86 boot_cpu_data;
extern struct cpuinfo_x86 new_cpu_data;

extern struct tss_struct doublefault_tss;
extern __u32 cleared_cpu_caps[8];


extern __typeof__(struct cpuinfo_x86) per_cpu__cpu_info;







static inline int hlt_works(int cpu)
{



 return 1;

}



extern void cpu_detect(struct cpuinfo_x86 *c);

extern void early_cpu_init(void);
extern void identify_boot_cpu(void);
extern void identify_secondary_cpu(struct cpuinfo_x86 *);
extern void print_cpu_info(struct cpuinfo_x86 *);
extern void init_scattered_cpuid_features(struct cpuinfo_x86 *c);
extern unsigned int init_intel_cacheinfo(struct cpuinfo_x86 *c);
extern unsigned short num_cache_leaves;


extern void detect_ht(struct cpuinfo_x86 *c);




static inline void native_cpuid(unsigned int *eax, unsigned int *ebx,
    unsigned int *ecx, unsigned int *edx)
{

 asm("cpuid"
     : "=a" (*eax),
       "=b" (*ebx),
       "=c" (*ecx),
       "=d" (*edx)
     : "0" (*eax), "2" (*ecx));
}

static inline void load_cr3(pgd_t *pgdir)
{
 write_cr3(__phys_addr((unsigned long)(pgdir)));
}
struct x86_hw_tss {
 u32 reserved1;
 u64 sp0;
 u64 sp1;
 u64 sp2;
 u64 reserved2;
 u64 ist[7];
 u32 reserved3;
 u32 reserved4;
 u16 reserved5;
 u16 io_bitmap_base;

} __attribute__((packed)) __attribute__((__aligned__((1 << (7)))));
struct tss_struct {



 struct x86_hw_tss x86_tss;







 unsigned long io_bitmap[((65536/8)/sizeof(long)) + 1];



 unsigned long io_bitmap_max;
 struct thread_struct *io_bitmap_owner;




 unsigned long stack[64];

} __attribute__((__aligned__((1 << (7)))));

extern __typeof__(struct tss_struct) per_cpu__init_tss;




struct orig_ist {
 unsigned long ist[7];
};



struct i387_fsave_struct {
 u32 cwd;
 u32 swd;
 u32 twd;
 u32 fip;
 u32 fcs;
 u32 foo;
 u32 fos;


 u32 st_space[20];


 u32 status;
};

struct i387_fxsave_struct {
 u16 cwd;
 u16 swd;
 u16 twd;
 u16 fop;
 union {
  struct {
   u64 rip;
   u64 rdp;
  };
  struct {
   u32 fip;
   u32 fcs;
   u32 foo;
   u32 fos;
  };
 };
 u32 mxcsr;
 u32 mxcsr_mask;


 u32 st_space[32];


 u32 xmm_space[64];

 u32 padding[24];

} __attribute__((aligned(16)));

struct i387_soft_struct {
 u32 cwd;
 u32 swd;
 u32 twd;
 u32 fip;
 u32 fcs;
 u32 foo;
 u32 fos;

 u32 st_space[20];
 u8 ftop;
 u8 changed;
 u8 lookahead;
 u8 no_update;
 u8 rm;
 u8 alimit;
 struct info *info;
 u32 entry_eip;
};

union thread_xstate {
 struct i387_fsave_struct fsave;
 struct i387_fxsave_struct fxsave;
 struct i387_soft_struct soft;
};


extern __typeof__(struct orig_ist) per_cpu__orig_ist;


extern void print_cpu_info(struct cpuinfo_x86 *);
extern unsigned int xstate_size;
extern void free_thread_xstate(struct task_struct *);
extern struct kmem_cache *task_xstate_cachep;
extern void init_scattered_cpuid_features(struct cpuinfo_x86 *c);
extern unsigned int init_intel_cacheinfo(struct cpuinfo_x86 *c);
extern unsigned short num_cache_leaves;

struct thread_struct {

 struct desc_struct tls_array[3];
 unsigned long sp0;
 unsigned long sp;



 unsigned long usersp;
 unsigned short es;
 unsigned short ds;
 unsigned short fsindex;
 unsigned short gsindex;

 unsigned long ip;
 unsigned long fs;
 unsigned long gs;

 unsigned long debugreg0;
 unsigned long debugreg1;
 unsigned long debugreg2;
 unsigned long debugreg3;
 unsigned long debugreg6;
 unsigned long debugreg7;

 unsigned long cr2;
 unsigned long trap_no;
 unsigned long error_code;

 union thread_xstate *xstate;
 unsigned long *io_bitmap_ptr;
 unsigned long iopl;

 unsigned io_bitmap_max;

 unsigned long debugctlmsr;


 unsigned long ds_area_msr;
};

static inline unsigned long native_get_debugreg(int regno)
{
 unsigned long val = 0;

 switch (regno) {
 case 0:
  asm("mov %%db0, %0" :"=r" (val));
  break;
 case 1:
  asm("mov %%db1, %0" :"=r" (val));
  break;
 case 2:
  asm("mov %%db2, %0" :"=r" (val));
  break;
 case 3:
  asm("mov %%db3, %0" :"=r" (val));
  break;
 case 6:
  asm("mov %%db6, %0" :"=r" (val));
  break;
 case 7:
  asm("mov %%db7, %0" :"=r" (val));
  break;
 default:
  ;
 }
 return val;
}

static inline void native_set_debugreg(int regno, unsigned long value)
{
 switch (regno) {
 case 0:
  asm("mov %0, %%db0" ::"r" (value));
  break;
 case 1:
  asm("mov %0, %%db1" ::"r" (value));
  break;
 case 2:
  asm("mov %0, %%db2" ::"r" (value));
  break;
 case 3:
  asm("mov %0, %%db3" ::"r" (value));
  break;
 case 6:
  asm("mov %0, %%db6" ::"r" (value));
  break;
 case 7:
  asm("mov %0, %%db7" ::"r" (value));
  break;
 default:
  ;
 }
}




static inline void native_set_iopl_mask(unsigned mask)
{
}

static inline void
native_load_sp0(struct tss_struct *tss, struct thread_struct *thread)
{
 tss->x86_tss.sp0 = thread->sp0;







}

static inline void native_swapgs(void)
{

 asm ("swapgs" ::: "memory");

}
extern unsigned long mmu_cr4_features;

static inline void set_in_cr4(unsigned long mask)
{
 unsigned cr4;

 mmu_cr4_features |= mask;
 cr4 = read_cr4();
 cr4 |= mask;
 write_cr4(cr4);
}

static inline void clear_in_cr4(unsigned long mask)
{
 unsigned cr4;

 mmu_cr4_features &= ~mask;
 cr4 = read_cr4();
 cr4 &= ~mask;
 write_cr4(cr4);
}

struct microcode_header {
 unsigned int hdrver;
 unsigned int rev;
 unsigned int date;
 unsigned int sig;
 unsigned int cksum;
 unsigned int ldrver;
 unsigned int pf;
 unsigned int datasize;
 unsigned int totalsize;
 unsigned int reserved[3];
};

struct microcode {
 struct microcode_header hdr;
 unsigned int bits[0];
};

typedef struct microcode microcode_t;
typedef struct microcode_header microcode_header_t;


struct extended_signature {
 unsigned int sig;
 unsigned int pf;
 unsigned int cksum;
};

struct extended_sigtable {
 unsigned int count;
 unsigned int cksum;
 unsigned int reserved[3];
 struct extended_signature sigs[0];
};

typedef struct {
 unsigned long seg;
} mm_segment_t;





extern int kernel_thread(int (*fn)(void *), void *arg, unsigned long flags);


extern void release_thread(struct task_struct *);


extern void prepare_to_copy(struct task_struct *tsk);

unsigned long get_wchan(struct task_struct *p);






static inline void cpuid(unsigned int op,
    unsigned int *eax, unsigned int *ebx,
    unsigned int *ecx, unsigned int *edx)
{
 *eax = op;
 *ecx = 0;
 __cpuid(eax, ebx, ecx, edx);
}


static inline void cpuid_count(unsigned int op, int count,
          unsigned int *eax, unsigned int *ebx,
          unsigned int *ecx, unsigned int *edx)
{
 *eax = op;
 *ecx = count;
 __cpuid(eax, ebx, ecx, edx);
}




static inline unsigned int cpuid_eax(unsigned int op)
{
 unsigned int eax, ebx, ecx, edx;

 cpuid(op, &eax, &ebx, &ecx, &edx);

 return eax;
}

static inline unsigned int cpuid_ebx(unsigned int op)
{
 unsigned int eax, ebx, ecx, edx;

 cpuid(op, &eax, &ebx, &ecx, &edx);

 return ebx;
}

static inline unsigned int cpuid_ecx(unsigned int op)
{
 unsigned int eax, ebx, ecx, edx;

 cpuid(op, &eax, &ebx, &ecx, &edx);

 return ecx;
}

static inline unsigned int cpuid_edx(unsigned int op)
{
 unsigned int eax, ebx, ecx, edx;

 cpuid(op, &eax, &ebx, &ecx, &edx);

 return edx;
}


static inline void rep_nop(void)
{
 asm ("rep; nop" ::: "memory");
}

static inline void cpu_relax(void)
{
 rep_nop();
}


static inline void sync_core(void)
{
 int tmp;

 asm ("cpuid" : "=a" (tmp) : "0" (1)
       : "ebx", "ecx", "edx", "memory");
}

static inline void __monitor( void *eax, unsigned long ecx,
        unsigned long edx)
{

 asm (".byte 0x0f, 0x01, 0xc8;"
       :: "a" (eax), "c" (ecx), "d"(edx));
}

static inline void __mwait(unsigned long eax, unsigned long ecx)
{

 asm (".byte 0x0f, 0x01, 0xc9;"
       :: "a" (eax), "c" (ecx));
}

static inline void __sti_mwait(unsigned long eax, unsigned long ecx)
{
 trace_hardirqs_on();

 asm ("sti; .byte 0x0f, 0x01, 0xc9;"
       :: "a" (eax), "c" (ecx));
}

extern void mwait_idle_with_hints(unsigned long eax, unsigned long ecx);

extern void select_idle_routine( struct cpuinfo_x86 *c);

extern unsigned long boot_option_idle_override;
extern unsigned long idle_halt;
extern unsigned long idle_nomwait;
static inline void wbinvd_halt(void)
{
 asm ("mfence":::"memory");

 if ((__builtin_constant_p((0*32+19)) && ( ((((0*32+19))>>5)==0 && (1UL<<(((0*32+19))&31) & ((1<<((0*32+ 0) & 31))|0|(1<<((0*32+ 5) & 31))|(1<<((0*32+ 6) & 31))| (1<<((0*32+ 8) & 31))|(1<<((0*32+13) & 31))|(1<<((0*32+24) & 31))|(1<<((0*32+15) & 31))| (1<<((0*32+25) & 31))|(1<<((0*32+26) & 31))))) || ((((0*32+19))>>5)==1 && (1UL<<(((0*32+19))&31) & ((1<<((1*32+29) & 31))|0))) || ((((0*32+19))>>5)==2 && (1UL<<(((0*32+19))&31) & 0)) || ((((0*32+19))>>5)==3 && (1UL<<(((0*32+19))&31) & ((1<<((3*32+20) & 31))))) || ((((0*32+19))>>5)==4 && (1UL<<(((0*32+19))&31) & 0)) || ((((0*32+19))>>5)==5 && (1UL<<(((0*32+19))&31) & 0)) || ((((0*32+19))>>5)==6 && (1UL<<(((0*32+19))&31) & 0)) || ((((0*32+19))>>5)==7 && (1UL<<(((0*32+19))&31) & 0)) ) ? 1 : (__builtin_constant_p(((0*32+19))) ? constant_test_bit(((0*32+19)), ((unsigned long *)((&boot_cpu_data)->x86_capability))) : variable_test_bit(((0*32+19)), ((unsigned long *)((&boot_cpu_data)->x86_capability))))))
  asm ("cli; wbinvd; 1: hlt; jmp 1b" : : : "memory");
 else
  while (1)
   halt();
}

extern void enable_sep_cpu(void);
extern int sysenter_setup(void);


extern struct desc_ptr early_gdt_descr;

extern void cpu_set_gdt(int);
extern void switch_to_new_gdt(void);
extern void cpu_init(void);
extern void init_gdt(int cpu);

static inline void update_debugctlmsr(unsigned long debugctlmsr)
{




 do { paravirt_write_msr(0x000001d9, (u32)((u64)(debugctlmsr)), ((u64)(debugctlmsr))>>32); } while (0);
}





extern unsigned int machine_id;
extern unsigned int machine_submodel_id;
extern unsigned int BIOS_revision;


extern int bootloader_type;

extern char ignore_fpu_irq;
static inline void prefetch( void *x)
{
 asm ("661:\n\t" "prefetcht0 (%1)" "\n662:\n" ".section .altinstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661b\n" " " ".quad" " " "663f\n" "	 .byte %c0\n" "	 .byte 662b-661b\n" "	 .byte 664f-663f\n" ".previous\n" ".section .altinstr_replacement,\"ax\"\n" "663:\n\t" "prefetchnta (%1)" "\n664:\n" ".previous" :: "i" ((0*32+25)), "r" (x))


             ;
}






static inline void prefetchw( void *x)
{
 asm ("661:\n\t" "prefetcht0 (%1)" "\n662:\n" ".section .altinstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661b\n" " " ".quad" " " "663f\n" "	 .byte %c0\n" "	 .byte 662b-661b\n" "	 .byte 664f-663f\n" ".previous\n" ".section .altinstr_replacement,\"ax\"\n" "663:\n\t" "prefetchw (%1)" "\n664:\n" ".previous" :: "i" ((1*32+31)), "r" (x))


             ;
}

static inline void spin_lock_prefetch( void *x)
{
 prefetchw(x);
}
extern void start_thread(struct pt_regs *regs, unsigned long new_ip,
            unsigned long new_sp);
extern int get_tsc_mode(unsigned long adr);
extern int set_tsc_mode(unsigned int val);

struct thread_info {
 struct task_struct *task;
 struct exec_domain *exec_domain;
 unsigned long flags;
 __u32 status;
 __u32 cpu;
 int preempt_count;

 mm_segment_t addr_limit;
 struct restart_block restart_block;
 void *sysenter_return;






};
static inline struct thread_info *current_thread_info(void)
{
 struct thread_info *ti;
 ti = (void *)(({ typeof(_proxy_pda.kernelstack) ret__; switch (sizeof(_proxy_pda.kernelstack)) { case 2: asm("mov" "w %%gs:%c1,%0" : "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->kernelstack)), "m" (_proxy_pda.kernelstack)); break; case 4: asm("mov" "l %%gs:%c1,%0": "=r" (ret__): "i" (((size_t) &((struct x8664_pda *)0)->kernelstack)), "m" (_proxy_pda.kernelstack)); break; case 8: asm("mov" "q %%gs:%c1,%0": "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->kernelstack)), "m" (_proxy_pda.kernelstack)); break; default: __bad_pda_field(); } ret__; }) + (5*8) - (((1UL) << 12) << 1));
 return ti;
}


static inline struct thread_info *stack_thread_info(void)
{
 struct thread_info *ti;
 asm("andq %%rsp,%0; " : "=r" (ti) : "0" (~((((1UL) << 12) << 1) - 1)));
 return ti;
}
static inline void set_restore_sigmask(void)
{
 struct thread_info *ti = current_thread_info();
 ti->status |= 0x0008;
 set_bit(2, (unsigned long *)&ti->flags);
}



extern void arch_task_cache_init(void);
extern void free_thread_info(struct thread_info *ti);
extern int arch_dup_task_struct(struct task_struct *dst, struct task_struct *src);
static inline void set_ti_thread_flag(struct thread_info *ti, int flag)
{
 set_bit(flag, (unsigned long *)&ti->flags);
}

static inline void clear_ti_thread_flag(struct thread_info *ti, int flag)
{
 clear_bit(flag, (unsigned long *)&ti->flags);
}

static inline int test_and_set_ti_thread_flag(struct thread_info *ti, int flag)
{
 return test_and_set_bit(flag, (unsigned long *)&ti->flags);
}

static inline int test_and_clear_ti_thread_flag(struct thread_info *ti, int flag)
{
 return test_and_clear_bit(flag, (unsigned long *)&ti->flags);
}

static inline int test_ti_thread_flag(struct thread_info *ti, int flag)
{
 return (__builtin_constant_p((flag)) ? constant_test_bit((flag), ((unsigned long *)&ti->flags)) : variable_test_bit((flag), ((unsigned long *)&ti->flags)));
}





static inline void prefetch_range(void *addr, size_t len)
{







}
struct list_head {
 struct list_head *next, *prev;
};






static inline void INIT_LIST_HEAD(struct list_head *list)
{
 list->next = list;
 list->prev = list;
}
extern void __list_add(struct list_head *_new,
         struct list_head *prev,
         struct list_head *next);
static inline void list_add(struct list_head *_new, struct list_head *head)
{
 __list_add(_new, head, head->next);
}
static inline void list_add_tail(struct list_head *_new, struct list_head *head)
{
 __list_add(_new, head->prev, head);
}
static inline void __list_del(struct list_head * prev, struct list_head * next)
{
 next->prev = prev;
 prev->next = next;
}
extern void list_del(struct list_head *entry);
static inline void list_replace(struct list_head *old,
    struct list_head *_new)
{
 _new->next = old->next;
 _new->next->prev = _new;
 _new->prev = old->prev;
 _new->prev->next = _new;
}

static inline void list_replace_init(struct list_head *old,
     struct list_head *_new)
{
 list_replace(old, _new);
 INIT_LIST_HEAD(old);
}





static inline void list_del_init(struct list_head *entry)
{
 __list_del(entry->prev, entry->next);
 INIT_LIST_HEAD(entry);
}






static inline void list_move(struct list_head *list, struct list_head *head)
{
 __list_del(list->prev, list->next);
 list_add(list, head);
}






static inline void list_move_tail(struct list_head *list,
      struct list_head *head)
{
 __list_del(list->prev, list->next);
 list_add_tail(list, head);
}






static inline int list_is_last( struct list_head *list,
    struct list_head *head)
{
 return list->next == head;
}





static inline int list_empty( struct list_head *head)
{
 return head->next == head;
}
static inline int list_empty_careful( struct list_head *head)
{
 struct list_head *next = head->next;
 return (next == head) && (next == head->prev);
}





static inline int list_is_singular( struct list_head *head)
{
 return !list_empty(head) && (head->next == head->prev);
}

static inline void __list_cut_position(struct list_head *list,
  struct list_head *head, struct list_head *entry)
{
 struct list_head *new_first = entry->next;
 list->next = head->next;
 list->next->prev = list;
 list->prev = entry;
 entry->next = list;
 head->next = new_first;
 new_first->prev = head;
}
static inline void list_cut_position(struct list_head *list,
  struct list_head *head, struct list_head *entry)
{
 if (list_empty(head))
  return;
 if (list_is_singular(head) &&
  (head->next != entry && head != entry))
  return;
 if (entry == head)
  INIT_LIST_HEAD(list);
 else
  __list_cut_position(list, head, entry);
}

static inline void __list_splice( struct list_head *list,
     struct list_head *prev,
     struct list_head *next)
{
 struct list_head *first = list->next;
 struct list_head *last = list->prev;

 first->prev = prev;
 prev->next = first;

 last->next = next;
 next->prev = last;
}






static inline void list_splice( struct list_head *list,
    struct list_head *head)
{
 if (!list_empty(list))
  __list_splice(list, head, head->next);
}






static inline void list_splice_tail(struct list_head *list,
    struct list_head *head)
{
 if (!list_empty(list))
  __list_splice(list, head->prev, head);
}
static inline void list_splice_init(struct list_head *list,
        struct list_head *head)
{
 if (!list_empty(list)) {
  __list_splice(list, head, head->next);
  INIT_LIST_HEAD(list);
 }
}
static inline void list_splice_tail_init(struct list_head *list,
      struct list_head *head)
{
 if (!list_empty(list)) {
  __list_splice(list, head->prev, head);
  INIT_LIST_HEAD(list);
 }
}
struct hlist_head {
 struct hlist_node *first;
};

struct hlist_node {
 struct hlist_node *next, **pprev;
};




static inline void INIT_HLIST_NODE(struct hlist_node *h)
{
 h->next = 0;
 h->pprev = 0;
}

static inline int hlist_unhashed( struct hlist_node *h)
{
 return !h->pprev;
}

static inline int hlist_empty( struct hlist_head *h)
{
 return !h->first;
}

static inline void __hlist_del(struct hlist_node *n)
{
 struct hlist_node *next = n->next;
 struct hlist_node **pprev = n->pprev;
 *pprev = next;
 if (next)
  next->pprev = pprev;
}

static inline void hlist_del(struct hlist_node *n)
{
 __hlist_del(n);
 n->next = ((void *) 0x00100100);
 n->pprev = ((void *) 0x00200200);
}

static inline void hlist_del_init(struct hlist_node *n)
{
 if (!hlist_unhashed(n)) {
  __hlist_del(n);
  INIT_HLIST_NODE(n);
 }
}

static inline void hlist_add_head(struct hlist_node *n, struct hlist_head *h)
{
 struct hlist_node *first = h->first;
 n->next = first;
 if (first)
  first->pprev = &n->next;
 h->first = n;
 n->pprev = &h->first;
}


static inline void hlist_add_before(struct hlist_node *n,
     struct hlist_node *next)
{
 n->pprev = next->pprev;
 n->next = next;
 next->pprev = &n->next;
 *(n->pprev) = n;
}

static inline void hlist_add_after(struct hlist_node *n,
     struct hlist_node *next)
{
 next->next = n->next;
 n->next = next;
 next->pprev = &n->next;

 if(next->next)
  next->next->pprev = &next->next;
}





static inline void hlist_move_list(struct hlist_head *old,
       struct hlist_head *_new)
{
 _new->first = old->first;
 if (_new->first)
  _new->first->pprev = &_new->first;
 old->first = 0;
}
struct preempt_notifier;
struct preempt_ops {
 void (*sched_in)(struct preempt_notifier *notifier, int cpu);
 void (*sched_out)(struct preempt_notifier *notifier,
     struct task_struct *next);
};
struct preempt_notifier {
 struct hlist_node link;
 struct preempt_ops *ops;
};

void preempt_notifier_register(struct preempt_notifier *notifier);
void preempt_notifier_unregister(struct preempt_notifier *notifier);

static inline void preempt_notifier_init(struct preempt_notifier *notifier,
         struct preempt_ops *ops)
{
 INIT_HLIST_NODE(&notifier->link);
 notifier->ops = ops;
}







extern void local_bh_disable(void);
extern void __local_bh_enable(void);
extern void _local_bh_enable(void);
extern void local_bh_enable(void);
extern void local_bh_enable_ip(unsigned long ip);







typedef struct raw_spinlock {
 unsigned int slock;
} raw_spinlock_t;



typedef struct {
 unsigned int lock;
} raw_rwlock_t;




struct task_struct;
struct lockdep_map;
static inline void lockdep_off(void)
{
}

static inline void lockdep_on(void)
{
}
struct lock_class_key { };
extern void early_init_irq_lock_class(void);







extern void early_boot_irqs_off(void);
extern void early_boot_irqs_on(void);
extern void print_irqtrace_events(struct task_struct *curr);

typedef struct {
 raw_spinlock_t raw_lock;
} spinlock_t;



typedef struct {
 raw_rwlock_t raw_lock;
} rwlock_t;

extern int __attribute__((section(".spinlock.text"))) generic__raw_read_trylock(raw_rwlock_t *lock);















typedef struct _atomic {
 int counter;
} atomic_t;
static inline int atomic_sub_return(int i, atomic_t *v)
{
 return 1;
}






typedef struct _atomic64 {
 long counter;
} atomic64_t;
static inline long atomic64_sub_return(long i, atomic64_t *v)
{
 return 1;
}
typedef atomic64_t atomic_long_t;



static inline long atomic_long_read(atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return (long)((v)->counter);
}

static inline void atomic_long_set(atomic_long_t *l, long i)
{
 atomic64_t *v = (atomic64_t *)l;

 ;
}

static inline void atomic_long_inc(atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 ;
}

static inline void atomic_long_dec(atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 ;
}

static inline void atomic_long_add(long i, atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 ;
}

static inline void atomic_long_sub(long i, atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 ;
}

static inline int atomic_long_sub_and_test(long i, atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return 1;
}

static inline int atomic_long_dec_and_test(atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return 1;
}

static inline int atomic_long_inc_and_test(atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return 1;
}

static inline int atomic_long_add_negative(long i, atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return 1;
}

static inline long atomic_long_add_return(long i, atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return (long)1;
}

static inline long atomic_long_sub_return(long i, atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return (long)atomic64_sub_return(i, v);
}

static inline long atomic_long_inc_return(atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return (long)(1);
}

static inline long atomic_long_dec_return(atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return (long)(atomic64_sub_return(1, (v)));
}

static inline long atomic_long_add_unless(atomic_long_t *l, long a, long u)
{
 atomic64_t *v = (atomic64_t *)l;

 return (long)1;
}
static inline int __ticket_spin_is_locked(raw_spinlock_t *lock)
{
 int tmp = (*( typeof(lock->slock) *)&(lock->slock));

 return (((tmp >> 8) & 0xff) != (tmp & 0xff));
}

static inline int __ticket_spin_is_contended(raw_spinlock_t *lock)
{
 int tmp = (*( typeof(lock->slock) *)&(lock->slock));

 return (((tmp >> 8) - tmp) & 0xff) > 1;
}

static inline __attribute__((always_inline)) void __ticket_spin_lock(raw_spinlock_t *lock)
{
 short inc = 0x0100;

 asm (
  ".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "xaddw %w0, %1\n"
  "1:\t"
  "cmpb %h0, %b0\n\t"
  "je 2f\n\t"
  "rep ; nop\n\t"
  "movb %1, %b0\n\t"

  "jmp 1b\n"
  "2:"
  : "+Q" (inc), "+m" (lock->slock)
  :
  : "memory", "cc");
}

static inline __attribute__((always_inline)) int __ticket_spin_trylock(raw_spinlock_t *lock)
{
 int tmp;
 short _new;

 asm ("movw %2,%w0\n\t"
       "cmpb %h0,%b0\n\t"
       "jne 1f\n\t"
       "movw %w0,%w1\n\t"
       "incb %h1\n\t"
       "lock ; cmpxchgw %w1,%2\n\t"
       "1:"
       "sete %b1\n\t"
       "movzbl %b1,%0\n\t"
       : "=&a" (tmp), "=Q" (_new), "+m" (lock->slock)
       :
       : "memory", "cc");

 return tmp;
}

static inline __attribute__((always_inline)) void __ticket_spin_unlock(raw_spinlock_t *lock)
{
 asm ( "incb %0"
       : "+m" (lock->slock)
       :
       : "memory", "cc");
}
struct __byte_spinlock {
 s8 lock;
 s8 spinners;
};

static inline int __byte_spin_is_locked(raw_spinlock_t *lock)
{
 struct __byte_spinlock *bl = (struct __byte_spinlock *)lock;
 return bl->lock != 0;
}

static inline int __byte_spin_is_contended(raw_spinlock_t *lock)
{
 struct __byte_spinlock *bl = (struct __byte_spinlock *)lock;
 return bl->spinners != 0;
}

static inline void __byte_spin_lock(raw_spinlock_t *lock)
{
 struct __byte_spinlock *bl = (struct __byte_spinlock *)lock;
 s8 val = 1;

 asm("1: xchgb %1, %0\n"
     "   test %1,%1\n"
     "   jz 3f\n"
     "   " ".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "incb %2\n"
     "2: rep;nop\n"
     "   cmpb $1, %0\n"
     "   je 2b\n"
     "   " ".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "decb %2\n"
     "   jmp 1b\n"
     "3:"
     : "+m" (bl->lock), "+q" (val), "+m" (bl->spinners): : "memory");
}

static inline int __byte_spin_trylock(raw_spinlock_t *lock)
{
 struct __byte_spinlock *bl = (struct __byte_spinlock *)lock;
 u8 old = 1;

 asm("xchgb %1,%0"
     : "+m" (bl->lock), "+q" (old) : : "memory");

 return old == 0;
}

static inline void __byte_spin_unlock(raw_spinlock_t *lock)
{
 struct __byte_spinlock *bl = (struct __byte_spinlock *)lock;
 ;
 bl->lock = 0;
}
static inline void __raw_spin_unlock_wait(raw_spinlock_t *lock)
{
 while (__raw_spin_is_locked(lock))
  cpu_relax();
}
static inline int __raw_read_can_lock(raw_rwlock_t *lock)
{
 return (int)(lock)->lock > 0;
}





static inline int __raw_write_can_lock(raw_rwlock_t *lock)
{
 return (lock)->lock == 0x01000000;
}

static inline void __raw_read_lock(raw_rwlock_t *rw)
{
 asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " " subl $1,(%0)\n\t"
       "jns 1f\n"
       "call __read_lock_failed\n\t"
       "1:\n"
       ::"D" (rw) : "memory");
}

static inline void __raw_write_lock(raw_rwlock_t *rw)
{
 asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " " subl %1,(%0)\n\t"
       "jz 1f\n"
       "call __write_lock_failed\n\t"
       "1:\n"
       ::"D" (rw), "i" (0x01000000) : "memory");
}

static inline int __raw_read_trylock(raw_rwlock_t *lock)
{
 atomic_t *count = (atomic_t *)lock;

 ;
 if (((count)->counter) >= 0)
  return 1;
 ;
 return 0;
}

static inline int __raw_write_trylock(raw_rwlock_t *lock)
{
 atomic_t *count = (atomic_t *)lock;

 if (1)
  return 1;
 ;
 return 0;
}

static inline void __raw_read_unlock(raw_rwlock_t *rw)
{
 asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "incl %0" :"+m" (rw->lock) : : "memory");
}

static inline void __raw_write_unlock(raw_rwlock_t *rw)
{
 asm (".section .smp_locks,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661f\n" ".previous\n" "661:\n\tlock; " "addl %1, %0"
       : "+m" (rw->lock) : "i" (0x01000000) : "memory");
}
int in_lock_functions(unsigned long addr);



void __attribute__((section(".spinlock.text"))) _spin_lock(spinlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _spin_lock_nested(spinlock_t *lock, int subclass)
       ;
void __attribute__((section(".spinlock.text"))) _spin_lock_nest_lock(spinlock_t *lock, struct lockdep_map *map)
       ;
void __attribute__((section(".spinlock.text"))) _read_lock(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _write_lock(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _spin_lock_bh(spinlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _read_lock_bh(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _write_lock_bh(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _spin_lock_irq(spinlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _read_lock_irq(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _write_lock_irq(rwlock_t *lock) ;
unsigned long __attribute__((section(".spinlock.text"))) _spin_lock_irqsave(spinlock_t *lock)
       ;
unsigned long __attribute__((section(".spinlock.text"))) _spin_lock_irqsave_nested(spinlock_t *lock, int subclass)
       ;
unsigned long __attribute__((section(".spinlock.text"))) _read_lock_irqsave(rwlock_t *lock)
       ;
unsigned long __attribute__((section(".spinlock.text"))) _write_lock_irqsave(rwlock_t *lock)
       ;
int __attribute__((section(".spinlock.text"))) _spin_trylock(spinlock_t *lock);
int __attribute__((section(".spinlock.text"))) _read_trylock(rwlock_t *lock);
int __attribute__((section(".spinlock.text"))) _write_trylock(rwlock_t *lock);
int __attribute__((section(".spinlock.text"))) _spin_trylock_bh(spinlock_t *lock);
void __attribute__((section(".spinlock.text"))) _spin_unlock(spinlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _read_unlock(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _write_unlock(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _spin_unlock_bh(spinlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _read_unlock_bh(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _write_unlock_bh(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _spin_unlock_irq(spinlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _read_unlock_irq(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _write_unlock_irq(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _spin_unlock_irqrestore(spinlock_t *lock, unsigned long flags)
       ;
void __attribute__((section(".spinlock.text"))) _read_unlock_irqrestore(rwlock_t *lock, unsigned long flags)
       ;
void __attribute__((section(".spinlock.text"))) _write_unlock_irqrestore(rwlock_t *lock, unsigned long flags)
       ;
extern int _atomic_dec_and_lock(atomic_t *atomic, spinlock_t *lock);


typedef struct {
 unsigned sequence;
 spinlock_t lock;
} seqlock_t;
static inline void write_seqlock(seqlock_t *sl)
{
 ;
 ++sl->sequence;
 ;
}

static inline void write_sequnlock(seqlock_t *sl)
{
 ;
 sl->sequence++;
 ;
}

static inline int write_tryseqlock(seqlock_t *sl)
{
 int ret = 1;

 if (ret) {
  ++sl->sequence;
  ;
 }
 return ret;
}
typedef struct seqcount {
 unsigned sequence;
} seqcount_t;





static inline unsigned read_seqcount_begin( seqcount_t *s)
{
 unsigned ret;

repeat:
 ret = s->sequence;
 ;
 if (__builtin_expect(!!(ret & 1), 0)) {
  cpu_relax();
  goto repeat;
 }
 return ret;
}




static inline int read_seqcount_retry( seqcount_t *s, unsigned start)
{
 ;

 return s->sequence != start;
}






static inline void write_seqcount_begin(seqcount_t *s)
{
 s->sequence++;
 ;
}

static inline void write_seqcount_end(seqcount_t *s)
{
 ;
 s->sequence++;
}




static inline u64 div_u64_rem(u64 dividend, u32 divisor, u32 *remainder)
{
 *remainder = dividend % divisor;
 return dividend / divisor;
}




static inline s64 div_s64_rem(s64 dividend, s32 divisor, s32 *remainder)
{
 *remainder = dividend % divisor;
 return dividend / divisor;
}




static inline u64 div64_u64(u64 dividend, u64 divisor)
{
 return dividend / divisor;
}
static inline u64 div_u64(u64 dividend, u32 divisor)
{
 u32 remainder;
 return div_u64_rem(dividend, divisor, &remainder);
}






static inline s64 div_s64(s64 dividend, s32 divisor)
{
 s32 remainder;
 return div_s64_rem(dividend, divisor, &remainder);
}


u32 iter_div_u64_rem(u64 dividend, u32 divisor, u64 *remainder);

static inline __attribute__((always_inline)) u32
__iter_div_u64_rem(u64 dividend, u32 divisor, u64 *remainder)
{
 u32 ret = 0;

 while (dividend >= divisor) {


  asm("" : "+rm"(dividend));

  dividend -= divisor;
  ret++;
 }

 *remainder = dividend;

 return ret;
}




struct timespec {
 time_t tv_sec;
 long tv_nsec;
};


struct timeval {
 time_t tv_sec;
 suseconds_t tv_usec;
};

struct timezone {
 int tz_minuteswest;
 int tz_dsttime;
};
static inline int timespec_equal( struct timespec *a,
                                 struct timespec *b)
{
 return (a->tv_sec == b->tv_sec) && (a->tv_nsec == b->tv_nsec);
}






static inline int timespec_compare( struct timespec *lhs, struct timespec *rhs)
{
 if (lhs->tv_sec < rhs->tv_sec)
  return -1;
 if (lhs->tv_sec > rhs->tv_sec)
  return 1;
 return lhs->tv_nsec - rhs->tv_nsec;
}

static inline int timeval_compare( struct timeval *lhs, struct timeval *rhs)
{
 if (lhs->tv_sec < rhs->tv_sec)
  return -1;
 if (lhs->tv_sec > rhs->tv_sec)
  return 1;
 return lhs->tv_usec - rhs->tv_usec;
}

extern unsigned long mktime( unsigned int year, unsigned int mon,
       unsigned int day, unsigned int hour,
       unsigned int min, unsigned int sec);

extern void set_normalized_timespec(struct timespec *ts, time_t sec, long nsec);




static inline struct timespec timespec_sub(struct timespec lhs,
      struct timespec rhs)
{
 struct timespec ts_delta;
 set_normalized_timespec(&ts_delta, lhs.tv_sec - rhs.tv_sec,
    lhs.tv_nsec - rhs.tv_nsec);
 return ts_delta;
}







extern struct timespec xtime;
extern struct timespec wall_to_monotonic;
extern seqlock_t xtime_lock;

extern unsigned long read_persistent_clock(void);
extern int update_persistent_clock(struct timespec now);
extern int no_sync_cmos_clock __attribute__((__section__(".data.read_mostly")));
void timekeeping_init(void);
extern int timekeeping_suspended;

unsigned long get_seconds(void);
struct timespec current_kernel_time(void);




extern void do_gettimeofday(struct timeval *tv);
extern int do_settimeofday(struct timespec *tv);
extern int do_sys_settimeofday(struct timespec *tv, struct timezone *tz);

extern long do_utimes(int dfd, char *filename, struct timespec *times, int flags);
struct itimerval;
extern int do_setitimer(int which, struct itimerval *value,
   struct itimerval *ovalue);
extern unsigned int alarm_setitimer(unsigned int seconds);
extern int do_getitimer(int which, struct itimerval *value);
extern void getnstimeofday(struct timespec *tv);
extern void getboottime(struct timespec *ts);
extern void monotonic_to_bootbased(struct timespec *ts);

extern struct timespec timespec_trunc(struct timespec t, unsigned gran);
extern int timekeeping_valid_for_hres(void);
extern void update_wall_time(void);
extern void update_xtime_cache(u64 nsec);
static inline s64 timespec_to_ns( struct timespec *ts)
{
 return ((s64) ts->tv_sec * 1000000000L) + ts->tv_nsec;
}
static inline s64 timeval_to_ns( struct timeval *tv)
{
 return ((s64) tv->tv_sec * 1000000000L) +
  tv->tv_usec * 1000L;
}







extern struct timespec ns_to_timespec( s64 nsec);







extern struct timeval ns_to_timeval( s64 nsec);
static inline __attribute__((always_inline)) void timespec_add_ns(struct timespec *a, u64 ns)
{
 a->tv_sec += __iter_div_u64_rem(a->tv_nsec + ns, 1000000000L, &ns);
 a->tv_nsec = ns;
}
struct itimerspec {
 struct timespec it_interval;
 struct timespec it_value;
};

struct itimerval {
 struct timeval it_interval;
 struct timeval it_value;
};
struct timex {
 unsigned int modes;
 long offset;
 long freq;
 long maxerror;
 long esterror;
 int status;
 long constant;
 long precision;
 long tolerance;


 struct timeval time;
 long tick;

 long ppsfreq;
 long jitter;
 int shift;
 long stabil;
 long jitcnt;
 long calcnt;
 long errcnt;
 long stbcnt;

 int tai;

 int :32; int :32; int :32; int :32;
 int :32; int :32; int :32; int :32;
 int :32; int :32; int :32;
};





typedef unsigned long long cycles_t;

extern unsigned int cpu_khz;
extern unsigned int tsc_khz;

extern void disable_TSC(void);

static inline cycles_t get_cycles(void)
{
 unsigned long long ret = 0;





 (ret = paravirt_read_tsc());

 return ret;
}

static inline __attribute__((always_inline)) cycles_t vget_cycles(void)
{
 return (cycles_t)__native_read_tsc();
}

extern void tsc_init(void);
extern void mark_tsc_unstable(char *reason);
extern int unsynchronized_tsc(void);
int check_tsc_unstable(void);





extern void check_tsc_sync_source(int cpu);
extern void check_tsc_sync_target(void);

extern int notsc_setup(char *);






extern unsigned long tick_usec;
extern unsigned long tick_nsec;
extern int tickadj;




extern int time_status;
extern long time_maxerror;
extern long time_esterror;

extern long time_adjust;

extern void ntp_init(void);
extern void ntp_clear(void);





static inline int ntp_synced(void)
{
 return !(time_status & 0x0040);
}
extern u64 tick_length;

extern void second_overflow(void);
extern void update_ntp_one_tick(void);
extern int do_adjtimex(struct timex *);




int read_current_timer(unsigned long *timer_val);
extern u64 __attribute__((section(".data"))) jiffies_64;
extern unsigned long __attribute__((section(".data"))) jiffies;




static inline u64 get_jiffies_64(void)
{
 return (u64)jiffies;
}
extern unsigned long preset_lpj;
extern unsigned int jiffies_to_msecs( unsigned long j);
extern unsigned int jiffies_to_usecs( unsigned long j);
extern unsigned long msecs_to_jiffies( unsigned int m);
extern unsigned long usecs_to_jiffies( unsigned int u);
extern unsigned long timespec_to_jiffies( struct timespec *value);
extern void jiffies_to_timespec( unsigned long jiffies,
    struct timespec *value);
extern unsigned long timeval_to_jiffies( struct timeval *value);
extern void jiffies_to_timeval( unsigned long jiffies,
          struct timeval *value);
extern clock_t jiffies_to_clock_t(long x);
extern unsigned long clock_t_to_jiffies(unsigned long x);
extern u64 jiffies_64_to_clock_t(u64 x);
extern u64 nsec_to_clock_t(u64 x);
struct rb_node
{
 unsigned long rb_parent_color;


 struct rb_node *rb_right;
 struct rb_node *rb_left;
} __attribute__((aligned(sizeof(long))));


struct rb_root
{
 struct rb_node *rb_node;
};
static inline void rb_set_parent(struct rb_node *rb, struct rb_node *p)
{
 rb->rb_parent_color = (rb->rb_parent_color & 3) | (unsigned long)p;
}
static inline void rb_set_color(struct rb_node *rb, int color)
{
 rb->rb_parent_color = (rb->rb_parent_color & ~1) | color;
}
extern void rb_insert_color(struct rb_node *, struct rb_root *);
extern void rb_erase(struct rb_node *, struct rb_root *);


extern struct rb_node *rb_next(struct rb_node *);
extern struct rb_node *rb_prev(struct rb_node *);
extern struct rb_node *rb_first(struct rb_root *);
extern struct rb_node *rb_last(struct rb_root *);


extern void rb_replace_node(struct rb_node *victim, struct rb_node *_new,
       struct rb_root *root);

static inline void rb_link_node(struct rb_node * node, struct rb_node * parent,
    struct rb_node ** rb_link)
{
 node->rb_parent_color = (unsigned long )parent;
 node->rb_left = node->rb_right = 0;

 *rb_link = node;
}






typedef struct { unsigned long bits[((((1 << 9)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))]; } nodemask_t;
extern nodemask_t _unused_nodemask_arg_;


static inline void __node_set(int node, nodemask_t *dstp)
{
 set_bit(node, dstp->bits);
}


static inline void __node_clear(int node, nodemask_t *dstp)
{
 clear_bit(node, dstp->bits);
}


static inline void __nodes_setall(nodemask_t *dstp, int nbits)
{
 bitmap_fill(dstp->bits, nbits);
}


static inline void __nodes_clear(nodemask_t *dstp, int nbits)
{
 bitmap_zero(dstp->bits, nbits);
}






static inline int __node_test_and_set(int node, nodemask_t *addr)
{
 return test_and_set_bit(node, addr->bits);
}



static inline void __nodes_and(nodemask_t *dstp, nodemask_t *src1p,
     nodemask_t *src2p, int nbits)
{
 bitmap_and(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline void __nodes_or(nodemask_t *dstp, nodemask_t *src1p,
     nodemask_t *src2p, int nbits)
{
 bitmap_or(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline void __nodes_xor(nodemask_t *dstp, nodemask_t *src1p,
     nodemask_t *src2p, int nbits)
{
 bitmap_xor(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline void __nodes_andnot(nodemask_t *dstp, nodemask_t *src1p,
     nodemask_t *src2p, int nbits)
{
 bitmap_andnot(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline void __nodes_complement(nodemask_t *dstp,
     nodemask_t *srcp, int nbits)
{
 bitmap_complement(dstp->bits, srcp->bits, nbits);
}



static inline int __nodes_equal( nodemask_t *src1p,
     nodemask_t *src2p, int nbits)
{
 return bitmap_equal(src1p->bits, src2p->bits, nbits);
}



static inline int __nodes_intersects( nodemask_t *src1p,
     nodemask_t *src2p, int nbits)
{
 return bitmap_intersects(src1p->bits, src2p->bits, nbits);
}



static inline int __nodes_subset( nodemask_t *src1p,
     nodemask_t *src2p, int nbits)
{
 return bitmap_subset(src1p->bits, src2p->bits, nbits);
}


static inline int __nodes_empty( nodemask_t *srcp, int nbits)
{
 return bitmap_empty(srcp->bits, nbits);
}


static inline int __nodes_full( nodemask_t *srcp, int nbits)
{
 return bitmap_full(srcp->bits, nbits);
}


static inline int __nodes_weight( nodemask_t *srcp, int nbits)
{
 return bitmap_weight(srcp->bits, nbits);
}



static inline void __nodes_shift_right(nodemask_t *dstp,
     nodemask_t *srcp, int n, int nbits)
{
 bitmap_shift_right(dstp->bits, srcp->bits, n, nbits);
}



static inline void __nodes_shift_left(nodemask_t *dstp,
     nodemask_t *srcp, int n, int nbits)
{
 bitmap_shift_left(dstp->bits, srcp->bits, n, nbits);
}





static inline int __first_node( nodemask_t *srcp)
{
 return ({ int __min1 = ((1 << 9)); int __min2 = (find_first_bit(srcp->bits, (1 << 9))); __min1 < __min2 ? __min1: __min2; });
}


static inline int __next_node(int n, nodemask_t *srcp)
{
 return ({ int __min1 = ((1 << 9)); int __min2 = (find_next_bit(srcp->bits, (1 << 9), n+1)); __min1 < __min2 ? __min1: __min2; });
}
static inline int __first_unset_node( nodemask_t *maskp)
{
 return ({ int __min1 = ((1 << 9)); int __min2 = (find_first_zero_bit(maskp->bits, (1 << 9))); __min1 < __min2 ? __min1: __min2; })
                                                  ;
}
static inline int __nodemask_scnprintf(char *buf, int len,
     nodemask_t *srcp, int nbits)
{
 return bitmap_scnprintf(buf, len, srcp->bits, nbits);
}



static inline int __nodemask_parse_user( char *buf, int len,
     nodemask_t *dstp, int nbits)
{
 return bitmap_parse_user(buf, len, dstp->bits, nbits);
}



static inline int __nodelist_scnprintf(char *buf, int len,
     nodemask_t *srcp, int nbits)
{
 return bitmap_scnlistprintf(buf, len, srcp->bits, nbits);
}


static inline int __nodelist_parse( char *buf, nodemask_t *dstp, int nbits)
{
 return bitmap_parselist(buf, dstp->bits, nbits);
}



static inline int __node_remap(int oldbit,
  nodemask_t *oldp, nodemask_t *newp, int nbits)
{
 return bitmap_bitremap(oldbit, oldp->bits, newp->bits, nbits);
}



static inline void __nodes_remap(nodemask_t *dstp, nodemask_t *srcp,
  nodemask_t *oldp, nodemask_t *newp, int nbits)
{
 bitmap_remap(dstp->bits, srcp->bits, oldp->bits, newp->bits, nbits);
}



static inline void __nodes_onto(nodemask_t *dstp, nodemask_t *origp,
  nodemask_t *relmapp, int nbits)
{
 bitmap_onto(dstp->bits, origp->bits, relmapp->bits, nbits);
}



static inline void __nodes_fold(nodemask_t *dstp, nodemask_t *origp,
  int sz, int nbits)
{
 bitmap_fold(dstp->bits, origp->bits, sz, nbits);
}
enum node_states {
 N_POSSIBLE,
 N_ONLINE,
 N_NORMAL_MEMORY,



 N_HIGH_MEMORY = N_NORMAL_MEMORY,

 N_CPU,
 NR_NODE_STATES
};






extern nodemask_t node_states[NR_NODE_STATES];


static inline int node_state(int node, enum node_states state)
{
 return (__builtin_constant_p(((node))) ? constant_test_bit(((node)), ((node_states[state]).bits)) : variable_test_bit(((node)), ((node_states[state]).bits)));
}

static inline void node_set_state(int node, enum node_states state)
{
 __node_set(node, &node_states[state]);
}

static inline void node_clear_state(int node, enum node_states state)
{
 __node_clear(node, &node_states[state]);
}

static inline int num_node_state(enum node_states state)
{
 return __nodes_weight(&(node_states[state]), (1 << 9));
}







extern int nr_node_ids;










struct raw_prio_tree_node {
 struct prio_tree_node *left;
 struct prio_tree_node *right;
 struct prio_tree_node *parent;
};

struct prio_tree_node {
 struct prio_tree_node *left;
 struct prio_tree_node *right;
 struct prio_tree_node *parent;
 unsigned long start;
 unsigned long last;
};

struct prio_tree_root {
 struct prio_tree_node *prio_tree_node;
 unsigned short index_bits;
 unsigned short raw;




};

struct prio_tree_iter {
 struct prio_tree_node *cur;
 unsigned long mask;
 unsigned long value;
 int size_level;

 struct prio_tree_root *root;
 unsigned long r_index;
 unsigned long h_index;
};

static inline void prio_tree_iter_init(struct prio_tree_iter *iter,
  struct prio_tree_root *root, unsigned long r_index, unsigned long h_index)
{
 iter->root = root;
 iter->r_index = r_index;
 iter->h_index = h_index;
 iter->cur = 0;
}
static inline int prio_tree_empty( struct prio_tree_root *root)
{
 return root->prio_tree_node == 0;
}

static inline int prio_tree_root( struct prio_tree_node *node)
{
 return node->parent == node;
}

static inline int prio_tree_left_empty( struct prio_tree_node *node)
{
 return node->left == node;
}

static inline int prio_tree_right_empty( struct prio_tree_node *node)
{
 return node->right == node;
}


struct prio_tree_node *prio_tree_replace(struct prio_tree_root *root,
                struct prio_tree_node *old, struct prio_tree_node *node);
struct prio_tree_node *prio_tree_insert(struct prio_tree_root *root,
                struct prio_tree_node *node);
void prio_tree_remove(struct prio_tree_root *root, struct prio_tree_node *node);
struct prio_tree_node *prio_tree_next(struct prio_tree_iter *iter);


struct rw_semaphore;


struct rwsem_waiter;
struct rw_semaphore {
 __s32 activity;
 spinlock_t wait_lock;
 struct list_head wait_list;



};
extern void __init_rwsem(struct rw_semaphore *sem, char *name,
    struct lock_class_key *key);
extern void __down_read(struct rw_semaphore *sem);
extern int __down_read_trylock(struct rw_semaphore *sem);
extern void __down_write(struct rw_semaphore *sem);
extern void __down_write_nested(struct rw_semaphore *sem, int subclass);
extern int __down_write_trylock(struct rw_semaphore *sem);
extern void __up_read(struct rw_semaphore *sem);
extern void __up_write(struct rw_semaphore *sem);
extern void __downgrade_write(struct rw_semaphore *sem);

static inline int rwsem_is_locked(struct rw_semaphore *sem)
{
 return (sem->activity != 0);
}
typedef struct __wait_queue wait_queue_t;
typedef int (*wait_queue_func_t)(wait_queue_t *wait, unsigned mode, int sync, void *key);
int default_wake_function(wait_queue_t *wait, unsigned mode, int sync, void *key);

struct __wait_queue {
 unsigned int flags;

 void *_private;
 wait_queue_func_t func;
 struct list_head task_list;
};

struct wait_bit_key {
 void *flags;
 int bit_nr;
};

struct wait_bit_queue {
 struct wait_bit_key key;
 wait_queue_t wait;
};

struct __wait_queue_head {
 spinlock_t lock;
 struct list_head task_list;
};
typedef struct __wait_queue_head wait_queue_head_t;

struct task_struct;
extern void init_waitqueue_head(wait_queue_head_t *q);
static inline void init_waitqueue_entry(wait_queue_t *q, struct task_struct *p)
{
 q->flags = 0;
 q->_private = p;
 q->func = default_wake_function;
}

static inline void init_waitqueue_func_entry(wait_queue_t *q,
     wait_queue_func_t func)
{
 q->flags = 0;
 q->_private = 0;
 q->func = func;
}

static inline int waitqueue_active(wait_queue_head_t *q)
{
 return !list_empty(&q->task_list);
}
extern void add_wait_queue(wait_queue_head_t *q, wait_queue_t *wait);
extern void add_wait_queue_exclusive(wait_queue_head_t *q, wait_queue_t *wait);
extern void remove_wait_queue(wait_queue_head_t *q, wait_queue_t *wait);

static inline void __add_wait_queue(wait_queue_head_t *head, wait_queue_t *_new)
{
 list_add(&_new->task_list, &head->task_list);
}




static inline void __add_wait_queue_tail(wait_queue_head_t *head,
      wait_queue_t *_new)
{
 list_add_tail(&_new->task_list, &head->task_list);
}

static inline void __remove_wait_queue(wait_queue_head_t *head,
       wait_queue_t *old)
{
 list_del(&old->task_list);
}

void __wake_up_common(wait_queue_head_t *q, unsigned int mode,
   int nr_exclusive, int sync, void *key);
void __wake_up(wait_queue_head_t *q, unsigned int mode, int nr, void *key);
extern void __wake_up_locked(wait_queue_head_t *q, unsigned int mode);
extern void __wake_up_sync(wait_queue_head_t *q, unsigned int mode, int nr);
void __wake_up_bit(wait_queue_head_t *, void *, int);
int __wait_on_bit(wait_queue_head_t *, struct wait_bit_queue *, int (*)(void *), unsigned);
int __wait_on_bit_lock(wait_queue_head_t *, struct wait_bit_queue *, int (*)(void *), unsigned);
void wake_up_bit(void *, int);
int out_of_line_wait_on_bit(void *, int, int (*)(void *), unsigned);
int out_of_line_wait_on_bit_lock(void *, int, int (*)(void *), unsigned);
wait_queue_head_t *bit_waitqueue(void *, int);
static inline void add_wait_queue_exclusive_locked(wait_queue_head_t *q,
         wait_queue_t * wait)
{
 wait->flags |= 0x01;
 __add_wait_queue_tail(q, wait);
}




static inline void remove_wait_queue_locked(wait_queue_head_t *q,
         wait_queue_t * wait)
{
 __remove_wait_queue(q, wait);
}






extern void sleep_on(wait_queue_head_t *q);
extern long sleep_on_timeout(wait_queue_head_t *q,
          signed long timeout);
extern void interruptible_sleep_on(wait_queue_head_t *q);
extern long interruptible_sleep_on_timeout(wait_queue_head_t *q,
        signed long timeout);




void prepare_to_wait(wait_queue_head_t *q, wait_queue_t *wait, int state);
void prepare_to_wait_exclusive(wait_queue_head_t *q, wait_queue_t *wait, int state);
void finish_wait(wait_queue_head_t *q, wait_queue_t *wait);
void abort_exclusive_wait(wait_queue_head_t *q, wait_queue_t *wait,
   unsigned int mode, void *key);
int autoremove_wake_function(wait_queue_t *wait, unsigned mode, int sync, void *key);
int wake_bit_function(wait_queue_t *wait, unsigned mode, int sync, void *key);
static inline int wait_on_bit(void *word, int bit,
    int (*action)(void *), unsigned mode)
{
        if (!(__builtin_constant_p((bit)) ? constant_test_bit((bit), ((unsigned long *)word)) : variable_test_bit((bit), ((unsigned long *)word))))
  return 0;
 return out_of_line_wait_on_bit(word, bit, action, mode);
}
static inline int wait_on_bit_lock(void *word, int bit,
    int (*action)(void *), unsigned mode)
{
 if (!test_and_set_bit(bit, (unsigned long *)word))
  return 0;
 return out_of_line_wait_on_bit_lock(word, bit, action, mode);
}

struct completion {
 unsigned int done;
 wait_queue_head_t wait;
};
static inline void init_completion(struct completion *x)
{
 x->done = 0;
 init_waitqueue_head(&x->wait);
}

extern void wait_for_completion(struct completion *);
extern int wait_for_completion_interruptible(struct completion *x);
extern int wait_for_completion_killable(struct completion *x);
extern unsigned long wait_for_completion_timeout(struct completion *x,
         unsigned long timeout);
extern unsigned long wait_for_completion_interruptible_timeout(
   struct completion *x, unsigned long timeout);
extern bool try_wait_for_completion(struct completion *x);
extern bool completion_done(struct completion *x);

extern void complete(struct completion *);
extern void complete_all(struct completion *);






struct mutex {

 atomic_t count;
 spinlock_t wait_lock;
 struct list_head wait_list;
};





struct mutex_waiter {
 struct list_head list;
 struct task_struct *task;




};







typedef struct {
 void *ldt;

 rwlock_t ldtlock;

 int size;
 struct mutex lock;
 void *vdso;
} mm_context_t;


void leave_mm(int cpu);






struct address_space;


typedef atomic_long_t mm_counter_t;
struct page {
 unsigned long flags;

 atomic_t _count;
 union {
  atomic_t _mapcount;



  struct {
   u16 inuse;
   u16 objects;
  };
 };
 union {
     struct {
  unsigned long _private;






  struct address_space *mapping;






     };

     spinlock_t ptl;

     struct kmem_cache *slab;
     struct page *first_page;
 };
 union {
  unsigned long index;
  void *freelist;
 };
 struct list_head lru;
};







struct vm_area_struct {
 struct mm_struct * vm_mm;
 unsigned long vm_start;
 unsigned long vm_end;



 struct vm_area_struct *vm_next;

 pgprot_t vm_page_prot;
 unsigned long vm_flags;

 struct rb_node vm_rb;







 union {
  struct {
   struct list_head list;
   void *parent;
   struct vm_area_struct *head;
  } vm_set;

  struct raw_prio_tree_node prio_tree_node;
 } shared;







 struct list_head anon_vma_node;
 struct anon_vma *anon_vma;


 struct vm_operations_struct * vm_ops;


 unsigned long vm_pgoff;

 struct file * vm_file;
 void * vm_private_data;
 unsigned long vm_truncate_count;





 struct mempolicy *vm_policy;

};

struct core_thread {
 struct task_struct *task;
 struct core_thread *next;
};

struct core_state {
 atomic_t nr_threads;
 struct core_thread dumper;
 struct completion startup;
};

struct mm_struct {
 struct vm_area_struct * mmap;
 struct rb_root mm_rb;
 struct vm_area_struct * mmap_cache;
 unsigned long (*get_unmapped_area) (struct file *filp,
    unsigned long addr, unsigned long len,
    unsigned long pgoff, unsigned long flags);
 void (*unmap_area) (struct mm_struct *mm, unsigned long addr);
 unsigned long mmap_base;
 unsigned long task_size;
 unsigned long cached_hole_size;
 unsigned long free_area_cache;
 pgd_t * pgd;
 atomic_t mm_users;
 atomic_t mm_count;
 int map_count;
 struct rw_semaphore mmap_sem;
 spinlock_t page_table_lock;

 struct list_head mmlist;







 mm_counter_t _file_rss;
 mm_counter_t _anon_rss;

 unsigned long hiwater_rss;
 unsigned long hiwater_vm;

 unsigned long total_vm, locked_vm, shared_vm, exec_vm;
 unsigned long stack_vm, reserved_vm, def_flags, nr_ptes;
 unsigned long start_code, end_code, start_data, end_data;
 unsigned long start_brk, brk, start_stack;
 unsigned long arg_start, arg_end, env_start, env_end;

 unsigned long saved_auxv[(2*(2 + 18 + 1))];

 cpumask_t cpu_vm_mask;


 mm_context_t context;
 unsigned int faultstamp;
 unsigned int token_priority;
 unsigned int last_interval;

 unsigned long flags;

 struct core_state *core_state;


 rwlock_t ioctx_list_lock;
 struct kioctx *ioctx_list;
 struct file *exe_file;
 unsigned long num_exe_file_vmas;


 struct mmu_notifier_mm *mmu_notifier_mm;

};










typedef unsigned long cputime_t;
typedef u64 cputime64_t;

extern void cpu_idle(void);

struct call_single_data {
 struct list_head list;
 void (*func) (void *info);
 void *info;
 unsigned int flags;
};












struct intel_mp_floating {
 char mpf_signature[4];
 unsigned int mpf_physptr;
 unsigned char mpf_length;
 unsigned char mpf_specification;
 unsigned char mpf_checksum;
 unsigned char mpf_feature1;
 unsigned char mpf_feature2;
 unsigned char mpf_feature3;
 unsigned char mpf_feature4;
 unsigned char mpf_feature5;
};



struct mp_config_table {
 char mpc_signature[4];
 unsigned short mpc_length;
 char mpc_spec;
 char mpc_checksum;
 char mpc_oem[8];
 char mpc_productid[12];
 unsigned int mpc_oemptr;
 unsigned short mpc_oemsize;
 unsigned short mpc_oemcount;
 unsigned int mpc_lapic;
 unsigned int reserved;
};
struct mpc_config_processor {
 unsigned char mpc_type;
 unsigned char mpc_apicid;
 unsigned char mpc_apicver;
 unsigned char mpc_cpuflag;
 unsigned int mpc_cpufeature;
 unsigned int mpc_featureflag;
 unsigned int mpc_reserved[2];
};

struct mpc_config_bus {
 unsigned char mpc_type;
 unsigned char mpc_busid;
 unsigned char mpc_bustype[6];
};
struct mpc_config_ioapic {
 unsigned char mpc_type;
 unsigned char mpc_apicid;
 unsigned char mpc_apicver;
 unsigned char mpc_flags;
 unsigned int mpc_apicaddr;
};

struct mpc_config_intsrc {
 unsigned char mpc_type;
 unsigned char mpc_irqtype;
 unsigned short mpc_irqflag;
 unsigned char mpc_srcbus;
 unsigned char mpc_srcbusirq;
 unsigned char mpc_dstapic;
 unsigned char mpc_dstirq;
};

enum mp_irq_source_types {
 mp_INT = 0,
 mp_NMI = 1,
 mp_SMI = 2,
 mp_ExtINT = 3
};







struct mpc_config_lintsrc {
 unsigned char mpc_type;
 unsigned char mpc_irqtype;
 unsigned short mpc_irqflag;
 unsigned char mpc_srcbusid;
 unsigned char mpc_srcbusirq;
 unsigned char mpc_destapic;
 unsigned char mpc_destapiclint;
};



struct mp_config_oemtable {
 char oem_signature[4];
 unsigned short oem_length;
 char oem_rev;
 char oem_checksum;
 char mpc_oem[8];
};
enum mp_bustype {
 MP_BUS_ISA = 1,
 MP_BUS_EISA,
 MP_BUS_PCI,
 MP_BUS_MCA,
};
extern void early_find_smp_config(void);
extern void early_get_smp_config(void);





extern unsigned long mp_bus_not_pci[(((256) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];

extern unsigned int boot_cpu_physical_apicid;
extern unsigned int max_physical_apicid;
extern int smp_found_config;
extern int mpc_default_type;
extern unsigned long mp_lapic_addr;

extern void find_smp_config(void);
extern void get_smp_config(void);

extern void early_reserve_e820_mpc_new(void);




void __attribute__ ((__section__(".cpuinit.text"))) __attribute__((__cold__)) generic_processor_info(int apicid, int version);

extern void mp_register_ioapic(int id, u32 address, u32 gsi_base);
extern void mp_override_legacy_irq(u8 bus_irq, u8 polarity, u8 trigger,
       u32 gsi);
extern void mp_config_acpi_legacy_irqs(void);
extern int mp_register_gsi(u32 gsi, int edge_level, int active_high_low);

extern int mp_config_acpi_gsi(unsigned char number, unsigned int devfn, u8 pin,
    u32 gsi, int triggering, int polarity);
struct physid_mask {
 unsigned long mask[(((255) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
};

typedef struct physid_mask physid_mask_t;
static inline void physid_set_mask_of_physid(int physid, physid_mask_t *map)
{
 bitmap_zero((*map).mask, 255);
 set_bit(physid, (*map).mask);
}




extern physid_mask_t phys_cpu_present_map;



extern void (*pm_idle)(void);
extern void (*pm_power_off)(void);
extern void (*pm_power_off_prepare)(void);





struct device;

typedef struct pm_message {
 int event;
} pm_message_t;
struct pm_ops {
 int (*prepare)(struct device *dev);
 void (*complete)(struct device *dev);
 int (*suspend)(struct device *dev);
 int (*resume)(struct device *dev);
 int (*freeze)(struct device *dev);
 int (*thaw)(struct device *dev);
 int (*poweroff)(struct device *dev);
 int (*restore)(struct device *dev);
};
struct pm_ext_ops {
 struct pm_ops base;
 int (*suspend_noirq)(struct device *dev);
 int (*resume_noirq)(struct device *dev);
 int (*freeze_noirq)(struct device *dev);
 int (*thaw_noirq)(struct device *dev);
 int (*poweroff_noirq)(struct device *dev);
 int (*restore_noirq)(struct device *dev);
};
enum dpm_state {
 DPM_INVALID,
 DPM_ON,
 DPM_PREPARING,
 DPM_RESUMING,
 DPM_SUSPENDING,
 DPM_OFF,
 DPM_OFF_IRQ,
};

struct dev_pm_info {
 pm_message_t power_state;
 unsigned can_wakeup:1;
 unsigned should_wakeup:1;
 enum dpm_state status;

 struct list_head entry;

};
extern void device_pm_lock(void);
extern void device_power_up(pm_message_t state);
extern void device_resume(pm_message_t state);

extern void device_pm_unlock(void);
extern int device_power_down(pm_message_t state);
extern int device_suspend(pm_message_t state);
extern int device_prepare_suspend(pm_message_t state);

extern void __suspend_report_result( char *function, void *fn, int ret);
extern unsigned int pm_flags;
extern unsigned long loops_per_jiffy;

extern void __bad_udelay(void);
extern void __bad_ndelay(void);

extern void __udelay(unsigned long usecs);
extern void __ndelay(unsigned long nsecs);
extern void __const_udelay(unsigned long xloops);
extern void __delay(unsigned long loops);
void use_tsc_delay(void);
extern unsigned long lpj_fine;
void calibrate_delay(void);
void msleep(unsigned int msecs);
unsigned long msleep_interruptible(unsigned int msecs);

static inline void ssleep(unsigned int seconds)
{
 msleep(seconds * 1000);
}
















struct local_apic {

        struct { unsigned int __reserved[4]; } __reserved_01;

        struct { unsigned int __reserved[4]; } __reserved_02;

        struct {
  unsigned int __reserved_1 : 24,
   phys_apic_id : 4,
   __reserved_2 : 4;
  unsigned int __reserved[3];
 } id;

       
 struct {
  unsigned int version : 8,
   __reserved_1 : 8,
   max_lvt : 8,
   __reserved_2 : 8;
  unsigned int __reserved[3];
 } version;

        struct { unsigned int __reserved[4]; } __reserved_03;

        struct { unsigned int __reserved[4]; } __reserved_04;

        struct { unsigned int __reserved[4]; } __reserved_05;

        struct { unsigned int __reserved[4]; } __reserved_06;

        struct {
  unsigned int priority : 8,
   __reserved_1 : 24;
  unsigned int __reserved_2[3];
 } tpr;

       
 struct {
  unsigned int priority : 8,
   __reserved_1 : 24;
  unsigned int __reserved_2[3];
 } apr;

       
 struct {
  unsigned int priority : 8,
   __reserved_1 : 24;
  unsigned int __reserved_2[3];
 } ppr;

        struct {
  unsigned int eoi;
  unsigned int __reserved[3];
 } eoi;

        struct { unsigned int __reserved[4]; } __reserved_07;

        struct {
  unsigned int __reserved_1 : 24,
   logical_dest : 8;
  unsigned int __reserved_2[3];
 } ldr;

        struct {
  unsigned int __reserved_1 : 28,
   model : 4;
  unsigned int __reserved_2[3];
 } dfr;

        struct {
  unsigned int spurious_vector : 8,
   apic_enabled : 1,
   focus_cpu : 1,
   __reserved_2 : 22;
  unsigned int __reserved_3[3];
 } svr;

        struct {
         unsigned int bitfield;
  unsigned int __reserved[3];
 } isr [8];

        struct {
         unsigned int bitfield;
  unsigned int __reserved[3];
 } tmr [8];

        struct {
         unsigned int bitfield;
  unsigned int __reserved[3];
 } irr [8];

        union {
  struct {
   unsigned int send_cs_error : 1,
    receive_cs_error : 1,
    send_accept_error : 1,
    receive_accept_error : 1,
    __reserved_1 : 1,
    send_illegal_vector : 1,
    receive_illegal_vector : 1,
    illegal_register_address : 1,
    __reserved_2 : 24;
   unsigned int __reserved_3[3];
  } error_bits;
  struct {
   unsigned int errors;
   unsigned int __reserved_3[3];
  } all_errors;
 } esr;

        struct { unsigned int __reserved[4]; } __reserved_08;

        struct { unsigned int __reserved[4]; } __reserved_09;

        struct { unsigned int __reserved[4]; } __reserved_10;

        struct { unsigned int __reserved[4]; } __reserved_11;

        struct { unsigned int __reserved[4]; } __reserved_12;

        struct { unsigned int __reserved[4]; } __reserved_13;

        struct { unsigned int __reserved[4]; } __reserved_14;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   destination_mode : 1,
   delivery_status : 1,
   __reserved_1 : 1,
   level : 1,
   trigger : 1,
   __reserved_2 : 2,
   shorthand : 2,
   __reserved_3 : 12;
  unsigned int __reserved_4[3];
 } icr1;

        struct {
  union {
   unsigned int __reserved_1 : 24,
    phys_dest : 4,
    __reserved_2 : 4;
   unsigned int __reserved_3 : 24,
    logical_dest : 8;
  } dest;
  unsigned int __reserved_4[3];
 } icr2;

        struct {
  unsigned int vector : 8,
   __reserved_1 : 4,
   delivery_status : 1,
   __reserved_2 : 3,
   mask : 1,
   timer_mode : 1,
   __reserved_3 : 14;
  unsigned int __reserved_4[3];
 } lvt_timer;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   __reserved_1 : 1,
   delivery_status : 1,
   __reserved_2 : 3,
   mask : 1,
   __reserved_3 : 15;
  unsigned int __reserved_4[3];
 } lvt_thermal;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   __reserved_1 : 1,
   delivery_status : 1,
   __reserved_2 : 3,
   mask : 1,
   __reserved_3 : 15;
  unsigned int __reserved_4[3];
 } lvt_pc;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   __reserved_1 : 1,
   delivery_status : 1,
   polarity : 1,
   remote_irr : 1,
   trigger : 1,
   mask : 1,
   __reserved_2 : 15;
  unsigned int __reserved_3[3];
 } lvt_lint0;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   __reserved_1 : 1,
   delivery_status : 1,
   polarity : 1,
   remote_irr : 1,
   trigger : 1,
   mask : 1,
   __reserved_2 : 15;
  unsigned int __reserved_3[3];
 } lvt_lint1;

        struct {
  unsigned int vector : 8,
   __reserved_1 : 4,
   delivery_status : 1,
   __reserved_2 : 3,
   mask : 1,
   __reserved_3 : 15;
  unsigned int __reserved_4[3];
 } lvt_error;

        struct {
  unsigned int initial_count;
  unsigned int __reserved_2[3];
 } timer_icr;

       
 struct {
  unsigned int curr_count;
  unsigned int __reserved_2[3];
 } timer_ccr;

        struct { unsigned int __reserved[4]; } __reserved_16;

        struct { unsigned int __reserved[4]; } __reserved_17;

        struct { unsigned int __reserved[4]; } __reserved_18;

        struct { unsigned int __reserved[4]; } __reserved_19;

        struct {
  unsigned int divisor : 4,
   __reserved_1 : 28;
  unsigned int __reserved_2[3];
 } timer_dcr;

        struct { unsigned int __reserved[4]; } __reserved_20;

} __attribute__ ((packed));

struct bootnode {
 u64 start;
 u64 end;
};

extern int compute_hash_shift(struct bootnode *nodes, int numblks,
         int *nodeids);



extern void numa_init_array(void);
extern int numa_off;

extern void srat_reserve_add_area(int nodeid);
extern int hotadd_percent;

extern s16 apicid_to_node[32768];

extern unsigned long numa_free_all_bootmem(void);
extern void setup_node_bootmem(int nodeid, unsigned long start,
          unsigned long end);


extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) init_cpu_to_node(void);
extern void __attribute__ ((__section__(".cpuinit.text"))) __attribute__((__cold__)) numa_set_node(int cpu, int node);
extern void __attribute__ ((__section__(".cpuinit.text"))) __attribute__((__cold__)) numa_clear_node(int cpu);
extern void __attribute__ ((__section__(".cpuinit.text"))) __attribute__((__cold__)) numa_add_cpu(int cpu);
extern void __attribute__ ((__section__(".cpuinit.text"))) __attribute__((__cold__)) numa_remove_cpu(int cpu);
int __acpi_acquire_global_lock(unsigned int *lock);
int __acpi_release_global_lock(unsigned int *lock);
extern int acpi_lapic;
extern int acpi_ioapic;
extern int acpi_noirq;
extern int acpi_strict;
extern int acpi_disabled;
extern int acpi_ht;
extern int acpi_pci_disabled;
extern int acpi_skip_timer_override;
extern int acpi_use_timer_override;

extern u8 acpi_sci_flags;
extern int acpi_sci_override_gsi;
void acpi_pic_sci_set_trigger(unsigned int, u16);

static inline void disable_acpi(void)
{
 acpi_disabled = 1;
 acpi_ht = 0;
 acpi_pci_disabled = 1;
 acpi_noirq = 1;
}




extern int acpi_gsi_to_irq(u32 gsi, unsigned int *irq);

static inline void acpi_noirq_set(void) { acpi_noirq = 1; }
static inline void acpi_disable_pci(void)
{
 acpi_pci_disabled = 1;
 acpi_noirq_set();
}
extern int acpi_irq_balance_set(char *str);


extern int acpi_save_state_mem(void);
extern void acpi_restore_state_mem(void);

extern unsigned long acpi_wakeup_address;


extern void acpi_reserve_bootmem(void);




static inline unsigned int acpi_processor_cstate_check(unsigned int max_cstate)
{






 if (boot_cpu_data.x86 == 0x0F &&
     boot_cpu_data.x86_vendor == 2 &&
     boot_cpu_data.x86_model <= 0x05 &&
     boot_cpu_data.x86_mask < 0x0A)
  return 1;
 else if ((__builtin_constant_p((3*32+21)) && ( ((((3*32+21))>>5)==0 && (1UL<<(((3*32+21))&31) & ((1<<((0*32+ 0) & 31))|0|(1<<((0*32+ 5) & 31))|(1<<((0*32+ 6) & 31))| (1<<((0*32+ 8) & 31))|(1<<((0*32+13) & 31))|(1<<((0*32+24) & 31))|(1<<((0*32+15) & 31))| (1<<((0*32+25) & 31))|(1<<((0*32+26) & 31))))) || ((((3*32+21))>>5)==1 && (1UL<<(((3*32+21))&31) & ((1<<((1*32+29) & 31))|0))) || ((((3*32+21))>>5)==2 && (1UL<<(((3*32+21))&31) & 0)) || ((((3*32+21))>>5)==3 && (1UL<<(((3*32+21))&31) & ((1<<((3*32+20) & 31))))) || ((((3*32+21))>>5)==4 && (1UL<<(((3*32+21))&31) & 0)) || ((((3*32+21))>>5)==5 && (1UL<<(((3*32+21))&31) & 0)) || ((((3*32+21))>>5)==6 && (1UL<<(((3*32+21))&31) & 0)) || ((((3*32+21))>>5)==7 && (1UL<<(((3*32+21))&31) & 0)) ) ? 1 : (__builtin_constant_p(((3*32+21))) ? constant_test_bit(((3*32+21)), ((unsigned long *)((&boot_cpu_data)->x86_capability))) : variable_test_bit(((3*32+21)), ((unsigned long *)((&boot_cpu_data)->x86_capability))))))
  return 1;
 else
  return max_cstate;
}
struct bootnode;


extern int acpi_numa;
extern int acpi_scan_nodes(unsigned long start, unsigned long end);

extern void acpi_fake_nodes( struct bootnode *fake_nodes,
       int num_nodes);





enum vsyscall_num {
 __NR_vgettimeofday,
 __NR_vtime,
 __NR_vgetcpu,
};
extern int __vgetcpu_mode;
extern unsigned long __jiffies;


extern int vgetcpu_mode;
extern struct timezone sys_tz;

extern void map_vsyscall(void);
extern u64 efi_call0(void *fp);
extern u64 efi_call1(void *fp, u64 arg1);
extern u64 efi_call2(void *fp, u64 arg1, u64 arg2);
extern u64 efi_call3(void *fp, u64 arg1, u64 arg2, u64 arg3);
extern u64 efi_call4(void *fp, u64 arg1, u64 arg2, u64 arg3, u64 arg4);
extern u64 efi_call5(void *fp, u64 arg1, u64 arg2, u64 arg3,
       u64 arg4, u64 arg5);
extern u64 efi_call6(void *fp, u64 arg1, u64 arg2, u64 arg3,
       u64 arg4, u64 arg5, u64 arg6);
extern void *efi_ioremap(unsigned long addr, unsigned long size);



extern void efi_reserve_early(void);
extern void efi_call_phys_prelog(void);
extern void efi_call_phys_epilog(void);
enum fixed_addresses {
 VSYSCALL_LAST_PAGE,
 VSYSCALL_FIRST_PAGE = VSYSCALL_LAST_PAGE
       + (((-2UL << 20)-(-10UL << 20)) >> 12) - 1,
 VSYSCALL_HPET,
 FIX_DBGP_BASE,
 FIX_EARLYCON_MEM_BASE,
 FIX_APIC_BASE,
 FIX_IO_APIC_BASE_0,
 FIX_IO_APIC_BASE_END = FIX_IO_APIC_BASE_0 + 128 - 1,
 FIX_EFI_IO_MAP_LAST_PAGE,
 FIX_EFI_IO_MAP_FIRST_PAGE = FIX_EFI_IO_MAP_LAST_PAGE
      + 100 - 1,

 FIX_PARAVIRT_BOOTMAP,


 FIX_ACPI_BEGIN,
 FIX_ACPI_END = FIX_ACPI_BEGIN + 4 - 1,


 FIX_OHCI1394_BASE,

 __end_of_permanent_fixed_addresses,
 FIX_BTMAP_END = __end_of_permanent_fixed_addresses + 512 -
   (__end_of_permanent_fixed_addresses & 511),
 FIX_BTMAP_BEGIN = FIX_BTMAP_END + 64*4 - 1,
 __end_of_fixed_addresses
};


extern int fixmaps_set;

void __native_set_fixmap(enum fixed_addresses idx, pte_t pte);
void native_set_fixmap(enum fixed_addresses idx,
         unsigned long phys, pgprot_t flags);
extern void __this_fixmap_does_not_exist(void);






static inline __attribute__((always_inline)) unsigned long fix_to_virt( unsigned int idx)
{
 if (idx >= __end_of_fixed_addresses)
  __this_fixmap_does_not_exist();

 return (((-2UL << 20)-((1UL) << 12)) - ((idx) << 12));
}

static inline unsigned long virt_to_fix( unsigned long vaddr)
{
 ;
 return ((((-2UL << 20)-((1UL) << 12)) - ((vaddr)&(~(((1UL) << 12)-1)))) >> 12);
}
extern void generic_apic_probe(void);



extern unsigned int apic_verbosity;
extern int local_apic_timer_c2_ok;

extern int ioapic_force;

extern int disable_apic;
extern int is_vsmp_box(void);

static inline void native_apic_write(unsigned long reg, u32 v)
{
 u32 *addr = ( u32 *)((fix_to_virt(FIX_APIC_BASE)) + reg);

 asm ("661:\n\t" "movl %0, %1" "\n662:\n" ".section .altinstructions,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "661b\n" " " ".quad" " " "663f\n" "	 .byte %c[feat]\n" "	 .byte 662b-661b\n" "	 .byte 664f-663f\n" ".previous\n" ".section .altinstr_replacement,\"ax\"\n" "663:\n\t" "xchgl %0, %1" "\n664:\n" ".previous" : "=r" (v), "=m" (*addr) : [feat] "i" ((3*32+19)), "0" (v), "m" (*addr))

                                           ;
}

static inline u32 native_apic_read(unsigned long reg)
{
 return *(( u32 *)((fix_to_virt(FIX_APIC_BASE)) + reg));
}

extern void apic_wait_icr_idle(void);
extern u32 safe_apic_wait_icr_idle(void);
extern int get_physical_broadcast(void);

static inline void ack_APIC_irq(void)
{
 apic_write(0xB0, 0);
}

extern int lapic_get_maxlvt(void);
extern void clear_local_APIC(void);
extern void connect_bsp_APIC(void);
extern void disconnect_bsp_APIC(int virt_wire_setup);
extern void disable_local_APIC(void);
extern void lapic_shutdown(void);
extern int verify_local_APIC(void);
extern void cache_APIC_registers(void);
extern void sync_Arb_IDs(void);
extern void init_bsp_APIC(void);
extern void setup_local_APIC(void);
extern void end_local_APIC_setup(void);
extern void init_apic_mappings(void);
extern void setup_boot_APIC_clock(void);
extern void setup_secondary_APIC_clock(void);
extern int APIC_init_uniprocessor(void);
extern void enable_NMI_through_LVT0(void);





extern void early_init_lapic_mapping(void);
extern int apic_is_clustered_box(void);







extern u8 setup_APIC_eilvt_mce(u8 vector, u8 msg_type, u8 mask);
extern u8 setup_APIC_eilvt_ibs(u8 vector, u8 msg_type, u8 mask);

union IO_APIC_reg_00 {
 u32 raw;
 struct {
  u32 __reserved_2 : 14,
   LTS : 1,
   delivery_type : 1,
   __reserved_1 : 8,
   ID : 8;
 } __attribute__ ((packed)) bits;
};

union IO_APIC_reg_01 {
 u32 raw;
 struct {
  u32 version : 8,
   __reserved_2 : 7,
   PRQ : 1,
   entries : 8,
   __reserved_1 : 8;
 } __attribute__ ((packed)) bits;
};

union IO_APIC_reg_02 {
 u32 raw;
 struct {
  u32 __reserved_2 : 24,
   arbitration : 4,
   __reserved_1 : 4;
 } __attribute__ ((packed)) bits;
};

union IO_APIC_reg_03 {
 u32 raw;
 struct {
  u32 boot_DT : 1,
   __reserved_1 : 31;
 } __attribute__ ((packed)) bits;
};

enum ioapic_irq_destination_types {
 dest_Fixed = 0,
 dest_LowestPrio = 1,
 dest_SMI = 2,
 dest__reserved_1 = 3,
 dest_NMI = 4,
 dest_INIT = 5,
 dest__reserved_2 = 6,
 dest_ExtINT = 7
};

struct IO_APIC_route_entry {
 __u32 vector : 8,
  delivery_mode : 3,



  dest_mode : 1,
  delivery_status : 1,
  polarity : 1,
  irr : 1,
  trigger : 1,
  mask : 1,
  __reserved_2 : 15;
 __u32 __reserved_3 : 24,
  dest : 8;


} __attribute__ ((packed));






extern int nr_ioapics;
extern int nr_ioapic_registers[128];







struct mp_config_ioapic {
 unsigned long mp_apicaddr;
 unsigned int mp_apicid;
 unsigned char mp_type;
 unsigned char mp_apicver;
 unsigned char mp_flags;
};

struct mp_config_intsrc {
 unsigned int mp_dstapic;
 unsigned char mp_type;
 unsigned char mp_irqtype;
 unsigned short mp_irqflag;
 unsigned char mp_srcbus;
 unsigned char mp_srcbusirq;
 unsigned char mp_dstirq;
};


extern struct mp_config_ioapic mp_ioapics[128];


extern int mp_irq_entries;


extern struct mp_config_intsrc mp_irqs[(256 * 4)];


extern int mpc_default_type;


extern int sis_apic_bug;


extern int skip_ioapic_setup;


extern int timer_through_8259;

static inline void disable_ioapic_setup(void)
{
 skip_ioapic_setup = 1;
}
extern int io_apic_get_unique_id(int ioapic, int apic_id);
extern int io_apic_get_version(int ioapic);
extern int io_apic_get_redir_entries(int ioapic);
extern int io_apic_set_pci_routing(int ioapic, int pin, int irq,
       int edge_level, int active_high_low);


extern int (*ioapic_renumber_irq)(int ioapic, int irq);
extern void ioapic_init_mappings(void);





extern cpumask_t cpu_callout_map;
extern cpumask_t cpu_initialized;
extern cpumask_t cpu_callin_map;

extern void (*mtrr_hook)(void);
extern void zap_low_mappings(void);

extern int __attribute__ ((__section__(".cpuinit.text"))) __attribute__((__cold__)) get_local_pda(int cpu);

extern int smp_num_siblings;
extern unsigned int num_processors;
extern cpumask_t cpu_initialized;

extern __typeof__(cpumask_t) per_cpu__cpu_sibling_map;
extern __typeof__(cpumask_t) per_cpu__cpu_core_map;
extern __typeof__(u16) per_cpu__cpu_llc_id;

extern __typeof__(u16) per_cpu__x86_cpu_to_apicid; extern __typeof__(u16) *x86_cpu_to_apicid_early_ptr; extern __typeof__(u16) x86_cpu_to_apicid_early_map[];
extern __typeof__(u16) per_cpu__x86_bios_cpu_apicid; extern __typeof__(u16) *x86_bios_cpu_apicid_early_ptr; extern __typeof__(u16) x86_bios_cpu_apicid_early_map[];


extern struct _stack_start {
 void *sp;
 unsigned short ss;
} stack_start;

struct smp_ops {
 void (*smp_prepare_boot_cpu)(void);
 void (*smp_prepare_cpus)(unsigned max_cpus);
 int (*cpu_up)(unsigned cpu);
 void (*smp_cpus_done)(unsigned max_cpus);

 void (*smp_send_stop)(void);
 void (*smp_send_reschedule)(int cpu);

 void (*send_call_func_ipi)(cpumask_t mask);
 void (*send_call_func_single_ipi)(int cpu);
};


extern void set_cpu_sibling_map(int cpu);





extern struct smp_ops smp_ops;

static inline void smp_send_stop(void)
{
 smp_ops.smp_send_stop();
}

static inline void smp_prepare_boot_cpu(void)
{
 smp_ops.smp_prepare_boot_cpu();
}

static inline void smp_prepare_cpus(unsigned int max_cpus)
{
 smp_ops.smp_prepare_cpus(max_cpus);
}

static inline void smp_cpus_done(unsigned int max_cpus)
{
 smp_ops.smp_cpus_done(max_cpus);
}

static inline int __cpu_up(unsigned int cpu)
{
 return smp_ops.cpu_up(cpu);
}

static inline void smp_send_reschedule(int cpu)
{
 smp_ops.smp_send_reschedule(cpu);
}

static inline void arch_send_call_function_single_ipi(int cpu)
{
 smp_ops.send_call_func_single_ipi(cpu);
}

static inline void arch_send_call_function_ipi(cpumask_t mask)
{
 smp_ops.send_call_func_ipi(mask);
}

void native_smp_prepare_boot_cpu(void);
void native_smp_prepare_cpus(unsigned int max_cpus);
void native_smp_cpus_done(unsigned int max_cpus);
int native_cpu_up(unsigned int cpunum);
void native_send_call_func_ipi(cpumask_t mask);
void native_send_call_func_single_ipi(int cpu);

extern int __cpu_disable(void);
extern void __cpu_die(unsigned int cpu);

void smp_store_cpu_info(int id);



static inline int num_booting_cpus(void)
{
 return __cpus_weight(&(cpu_callout_map), 64);
}



extern void prefill_possible_map(void);






extern unsigned disabled_cpus __attribute__ ((__section__(".cpuinit.data")));
static inline int logical_smp_processor_id(void)
{

 return (((*(u32 *)((fix_to_virt(FIX_APIC_BASE)) + 0xD0)) >> 24) & 0xFFu);
}







extern unsigned int read_apic_id(void);






static inline int hard_smp_processor_id(void)
{

 return (((read_apic_id())>>24)&0xFFu);
}
extern void cpu_uninit(void);
extern void smp_send_stop(void);




extern void smp_send_reschedule(int cpu);





extern void smp_prepare_cpus(unsigned int max_cpus);




extern int __cpu_up(unsigned int cpunum);




extern void smp_cpus_done(unsigned int max_cpus);




int smp_call_function(void(*func)(void *info), void *info, int wait);
int smp_call_function_mask(cpumask_t mask, void(*func)(void *info), void *info,
    int wait);
int smp_call_function_single(int cpuid, void (*func) (void *info), void *info,
    int wait);
void __smp_call_function_single(int cpuid, struct call_single_data *data);





void generic_smp_call_function_single_interrupt(void);
void generic_smp_call_function_interrupt(void);
void ipi_call_lock(void);
void ipi_call_unlock(void);
void ipi_call_lock_irq(void);
void ipi_call_unlock_irq(void);





int on_each_cpu(void (*func) (void *info), void *info, int wait);
void smp_prepare_boot_cpu(void);

extern unsigned int setup_max_cpus;
void smp_setup_processor_id(void);



struct ipc_perm
{
 __kernel_key_t key;
 __kernel_uid_t uid;
 __kernel_gid_t gid;
 __kernel_uid_t cuid;
 __kernel_gid_t cgid;
 __kernel_mode_t mode;
 unsigned short seq;
};


struct ipc64_perm {
 __kernel_key_t key;
 __kernel_uid32_t uid;
 __kernel_gid32_t gid;
 __kernel_uid32_t cuid;
 __kernel_gid32_t cgid;
 __kernel_mode_t mode;
 unsigned short __pad1;
 unsigned short seq;
 unsigned short __pad2;
 unsigned long __unused1;
 unsigned long __unused2;
};
struct ipc_kludge {
 struct msgbuf *msgp;
 long msgtyp;
};

struct kref {
 atomic_t refcount;
};

void kref_set(struct kref *kref, int num);
void kref_init(struct kref *kref);
void kref_get(struct kref *kref);
int kref_put(struct kref *kref, void (*release) (struct kref *kref));





struct kern_ipc_perm
{
 spinlock_t lock;
 int deleted;
 int id;
 key_t key;
 uid_t uid;
 gid_t gid;
 uid_t cuid;
 gid_t cgid;
 mode_t mode;
 unsigned long seq;
 void *security;
};
struct semid_ds {
 struct ipc_perm sem_perm;
 __kernel_time_t sem_otime;
 __kernel_time_t sem_ctime;
 struct sem *sem_base;
 struct sem_queue *sem_pending;
 struct sem_queue **sem_pending_last;
 struct sem_undo *undo;
 unsigned short sem_nsems;
};


struct semid64_ds {
 struct ipc64_perm sem_perm;
 __kernel_time_t sem_otime;
 unsigned long __unused1;
 __kernel_time_t sem_ctime;
 unsigned long __unused2;
 unsigned long sem_nsems;
 unsigned long __unused3;
 unsigned long __unused4;
};


struct sembuf {
 unsigned short sem_num;
 short sem_op;
 short sem_flg;
};


union semun {
 int val;
 struct semid_ds *buf;
 unsigned short *array;
 struct seminfo *__buf;
 void *__pad;
};

struct seminfo {
 int semmap;
 int semmni;
 int semmns;
 int semmnu;
 int semmsl;
 int semopm;
 int semume;
 int semusz;
 int semvmx;
 int semaem;
};







enum pageblock_bits {
 PB_migrate,
 PB_migrate_end = PB_migrate + 3 - 1,

 NR_PAGEBLOCK_BITS
};
struct page;


unsigned long get_pageblock_flags_group(struct page *page,
     int start_bitidx, int end_bitidx);
void set_pageblock_flags_group(struct page *page, unsigned long flags,
     int start_bitidx, int end_bitidx);
extern int page_group_by_mobility_disabled;

static inline int get_pageblock_migratetype(struct page *page)
{
 if (__builtin_expect(!!(page_group_by_mobility_disabled), 0))
  return 0;

 return get_pageblock_flags_group(page, PB_migrate, PB_migrate_end);
}

struct free_area {
 struct list_head free_list[5];
 unsigned long nr_free;
};

struct pglist_data;
struct zone_padding {
 char x[0];
} __attribute__((__aligned__(1 << ((7)))));





enum zone_stat_item {

 NR_FREE_PAGES,
 NR_INACTIVE,
 NR_ACTIVE,
 NR_ANON_PAGES,
 NR_FILE_MAPPED,

 NR_FILE_PAGES,
 NR_FILE_DIRTY,
 NR_WRITEBACK,

 NR_SLAB_RECLAIMABLE,
 NR_SLAB_UNRECLAIMABLE,
 NR_PAGETABLE,
 NR_UNSTABLE_NFS,
 NR_BOUNCE,
 NR_VMSCAN_WRITE,
 NR_WRITEBACK_TEMP,

 NUMA_HIT,
 NUMA_MISS,
 NUMA_FOREIGN,
 NUMA_INTERLEAVE_HIT,
 NUMA_LOCAL,
 NUMA_OTHER,

 NR_VM_ZONE_STAT_ITEMS };

struct per_cpu_pages {
 int count;
 int high;
 int batch;
 struct list_head list;
};

struct per_cpu_pageset {
 struct per_cpu_pages pcp;

 s8 expire;


 s8 stat_threshold;
 s8 vm_stat_diff[NR_VM_ZONE_STAT_ITEMS];

} __attribute__((__aligned__((1 << (7)))));
enum zone_type {
 ZONE_DMA,







 ZONE_DMA32,






 ZONE_NORMAL,
 ZONE_MOVABLE,
 __MAX_NR_ZONES
};
struct zone {

 unsigned long pages_min, pages_low, pages_high;
 unsigned long lowmem_reserve[4];


 int node;



 unsigned long min_unmapped_pages;
 unsigned long min_slab_pages;
 struct per_cpu_pageset *pageset[64];






 spinlock_t lock;




 struct free_area free_area[11];
 struct zone_padding _pad1_;


 spinlock_t lru_lock;
 struct list_head active_list;
 struct list_head inactive_list;
 unsigned long nr_scan_active;
 unsigned long nr_scan_inactive;
 unsigned long pages_scanned;
 unsigned long flags;


 atomic_long_t vm_stat[NR_VM_ZONE_STAT_ITEMS];
 int prev_priority;


 struct zone_padding _pad2_;
 wait_queue_head_t * wait_table;
 unsigned long wait_table_hash_nr_entries;
 unsigned long wait_table_bits;




 struct pglist_data *zone_pgdat;

 unsigned long zone_start_pfn;
 unsigned long spanned_pages;
 unsigned long present_pages;




 char *name;
} __attribute__((__aligned__(1 << ((7)))));

typedef enum {
 ZONE_ALL_UNRECLAIMABLE,
 ZONE_RECLAIM_LOCKED,
 ZONE_OOM_LOCKED,
} zone_flags_t;

static inline void zone_set_flag(struct zone *zone, zone_flags_t flag)
{
 set_bit(flag, &zone->flags);
}

static inline int zone_test_and_set_flag(struct zone *zone, zone_flags_t flag)
{
 return test_and_set_bit(flag, &zone->flags);
}

static inline void zone_clear_flag(struct zone *zone, zone_flags_t flag)
{
 clear_bit(flag, &zone->flags);
}

static inline int zone_is_all_unreclaimable( struct zone *zone)
{
 return (__builtin_constant_p((ZONE_ALL_UNRECLAIMABLE)) ? constant_test_bit((ZONE_ALL_UNRECLAIMABLE), (&zone->flags)) : variable_test_bit((ZONE_ALL_UNRECLAIMABLE), (&zone->flags)));
}

static inline int zone_is_reclaim_locked( struct zone *zone)
{
 return (__builtin_constant_p((ZONE_RECLAIM_LOCKED)) ? constant_test_bit((ZONE_RECLAIM_LOCKED), (&zone->flags)) : variable_test_bit((ZONE_RECLAIM_LOCKED), (&zone->flags)));
}

static inline int zone_is_oom_locked( struct zone *zone)
{
 return (__builtin_constant_p((ZONE_OOM_LOCKED)) ? constant_test_bit((ZONE_OOM_LOCKED), (&zone->flags)) : variable_test_bit((ZONE_OOM_LOCKED), (&zone->flags)));
}
struct zonelist_cache {
 unsigned short z_to_n[((1 << 9) * 4)];
 unsigned long fullzones[(((((1 << 9) * 4)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
 unsigned long last_full_zap;
};
struct zoneref {
 struct zone *zone;
 int zone_idx;
};
struct zonelist {
 struct zonelist_cache *zlcache_ptr;
 struct zoneref _zonerefs[((1 << 9) * 4) + 1];

 struct zonelist_cache zlcache;

};


struct node_active_region {
 unsigned long start_pfn;
 unsigned long end_pfn;
 int nid;
};




extern struct page *mem_map;
struct bootmem_data;
typedef struct pglist_data {
 struct zone node_zones[4];
 struct zonelist node_zonelists[2];
 int nr_zones;



 struct bootmem_data *bdata;
 unsigned long node_start_pfn;
 unsigned long node_present_pages;
 unsigned long node_spanned_pages;

 int node_id;
 wait_queue_head_t kswapd_wait;
 struct task_struct *kswapd;
 int kswapd_max_order;
} pg_data_t;




struct srcu_struct_array {
 int c[2];
};

struct srcu_struct {
 int completed;
 struct srcu_struct_array *per_cpu_ref;
 struct mutex mutex;
};







int init_srcu_struct(struct srcu_struct *sp);
void cleanup_srcu_struct(struct srcu_struct *sp);
int srcu_read_lock(struct srcu_struct *sp) ;
void srcu_read_unlock(struct srcu_struct *sp, int idx) ;
void synchronize_srcu(struct srcu_struct *sp);
long srcu_batches_completed(struct srcu_struct *sp);
struct notifier_block {
 int (*notifier_call)(struct notifier_block *, unsigned long, void *);
 struct notifier_block *next;
 int priority;
};

struct atomic_notifier_head {
 spinlock_t lock;
 struct notifier_block *head;
};

struct blocking_notifier_head {
 struct rw_semaphore rwsem;
 struct notifier_block *head;
};

struct raw_notifier_head {
 struct notifier_block *head;
};

struct srcu_notifier_head {
 struct mutex mutex;
 struct srcu_struct srcu;
 struct notifier_block *head;
};
extern void srcu_init_notifier_head(struct srcu_notifier_head *nh);
extern int atomic_notifier_chain_register(struct atomic_notifier_head *nh,
  struct notifier_block *nb);
extern int blocking_notifier_chain_register(struct blocking_notifier_head *nh,
  struct notifier_block *nb);
extern int raw_notifier_chain_register(struct raw_notifier_head *nh,
  struct notifier_block *nb);
extern int srcu_notifier_chain_register(struct srcu_notifier_head *nh,
  struct notifier_block *nb);

extern int blocking_notifier_chain_cond_register(
  struct blocking_notifier_head *nh,
  struct notifier_block *nb);

extern int atomic_notifier_chain_unregister(struct atomic_notifier_head *nh,
  struct notifier_block *nb);
extern int blocking_notifier_chain_unregister(struct blocking_notifier_head *nh,
  struct notifier_block *nb);
extern int raw_notifier_chain_unregister(struct raw_notifier_head *nh,
  struct notifier_block *nb);
extern int srcu_notifier_chain_unregister(struct srcu_notifier_head *nh,
  struct notifier_block *nb);

extern int atomic_notifier_call_chain(struct atomic_notifier_head *nh,
  unsigned long val, void *v);
extern int __atomic_notifier_call_chain(struct atomic_notifier_head *nh,
 unsigned long val, void *v, int nr_to_call, int *nr_calls);
extern int blocking_notifier_call_chain(struct blocking_notifier_head *nh,
  unsigned long val, void *v);
extern int __blocking_notifier_call_chain(struct blocking_notifier_head *nh,
 unsigned long val, void *v, int nr_to_call, int *nr_calls);
extern int raw_notifier_call_chain(struct raw_notifier_head *nh,
  unsigned long val, void *v);
extern int __raw_notifier_call_chain(struct raw_notifier_head *nh,
 unsigned long val, void *v, int nr_to_call, int *nr_calls);
extern int srcu_notifier_call_chain(struct srcu_notifier_head *nh,
  unsigned long val, void *v);
extern int __srcu_notifier_call_chain(struct srcu_notifier_head *nh,
 unsigned long val, void *v, int nr_to_call, int *nr_calls);
static inline int notifier_from_errno(int err)
{
 return 0x8000 | (0x0001 - err);
}


static inline int notifier_to_errno(int ret)
{
 ret &= ~0x8000;
 return ret > 0x0001 ? 0x0001 - ret : 0;
}
extern struct blocking_notifier_head reboot_notifier_list;

struct page;
struct zone;
struct pglist_data;
struct mem_section;
static inline void pgdat_resize_lock(struct pglist_data *p, unsigned long *f) {}
static inline void pgdat_resize_unlock(struct pglist_data *p, unsigned long *f) {}
static inline void pgdat_resize_init(struct pglist_data *pgdat) {}

static inline unsigned zone_span_seqbegin(struct zone *zone)
{
 return 0;
}
static inline int zone_span_seqretry(struct zone *zone, unsigned iv)
{
 return 0;
}
static inline void zone_span_writelock(struct zone *zone) {}
static inline void zone_span_writeunlock(struct zone *zone) {}
static inline void zone_seqlock_init(struct zone *zone) {}

static inline int mhp_notimplemented( char *func)
{
 printk("<4>" "%s() called, with CONFIG_MEMORY_HOTPLUG disabled\n", func);
 dump_stack();
 return -38;
}

static inline void register_page_bootmem_info_node(struct pglist_data *pgdat)
{
}







extern int walk_memory_resource(unsigned long start_pfn,
   unsigned long nr_pages, void *arg,
   int (*func)(unsigned long, unsigned long, void *));






static inline int is_mem_section_removable(unsigned long pfn,
     unsigned long nr_pages)
{
 return 0;
}


extern int add_memory(int nid, u64 start, u64 size);
extern int arch_add_memory(int nid, u64 start, u64 size);
extern int remove_memory(u64 start, u64 size);
extern int sparse_add_one_section(struct zone *zone, unsigned long start_pfn,
        int nr_pages);
extern void sparse_remove_one_section(struct zone *zone, struct mem_section *ms);
extern struct page *sparse_decode_mem_map(unsigned long coded_mem_map,
       unsigned long pnum);

void get_zone_counts(unsigned long *active, unsigned long *inactive,
   unsigned long *free);
void build_all_zonelists(void);
void wakeup_kswapd(struct zone *zone, int order);
int zone_watermark_ok(struct zone *z, int order, unsigned long mark,
  int classzone_idx, int alloc_flags);
enum memmap_context {
 MEMMAP_EARLY,
 MEMMAP_HOTPLUG,
};
extern int init_currently_empty_zone(struct zone *zone, unsigned long start_pfn,
         unsigned long size,
         enum memmap_context context);


void memory_present(int nid, unsigned long start, unsigned long end);
static inline int populated_zone(struct zone *zone)
{
 return (!!zone->present_pages);
}

extern int movable_zone;

static inline int zone_movable_is_highmem(void)
{



 return 0;

}

static inline int is_highmem_idx(enum zone_type idx)
{




 return 0;

}

static inline int is_normal_idx(enum zone_type idx)
{
 return (idx == ZONE_NORMAL);
}







static inline int is_highmem(struct zone *zone)
{






 return 0;

}

static inline int is_normal(struct zone *zone)
{
 return zone == zone->zone_pgdat->node_zones + ZONE_NORMAL;
}

static inline int is_dma32(struct zone *zone)
{

 return zone == zone->zone_pgdat->node_zones + ZONE_DMA32;



}

static inline int is_dma(struct zone *zone)
{

 return zone == zone->zone_pgdat->node_zones + ZONE_DMA;



}


struct ctl_table;
struct file;
int min_free_kbytes_sysctl_handler(struct ctl_table *, int, struct file *,
     void *, size_t *, loff_t *);
extern int sysctl_lowmem_reserve_ratio[4 -1];
int lowmem_reserve_ratio_sysctl_handler(struct ctl_table *, int, struct file *,
     void *, size_t *, loff_t *);
int percpu_pagelist_fraction_sysctl_handler(struct ctl_table *, int, struct file *,
     void *, size_t *, loff_t *);
int sysctl_min_unmapped_ratio_sysctl_handler(struct ctl_table *, int,
   struct file *, void *, size_t *, loff_t *);
int sysctl_min_slab_ratio_sysctl_handler(struct ctl_table *, int,
   struct file *, void *, size_t *, loff_t *);

extern int numa_zonelist_order_handler(struct ctl_table *, int,
   struct file *, void *, size_t *, loff_t *);
extern char numa_zonelist_order[];


extern cpumask_t *node_to_cpumask_map;


extern __typeof__(int) per_cpu__x86_cpu_to_node_map; extern __typeof__(int) *x86_cpu_to_node_map_early_ptr; extern __typeof__(int) x86_cpu_to_node_map_early_map[];
static inline int cpu_to_node(int cpu)
{
 return (*({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((&per_cpu__x86_cpu_to_node_map))); (typeof((&per_cpu__x86_cpu_to_node_map))) (__ptr + (((((_cpu_pda[cpu])->data_offset))))); }));
}


static inline int early_cpu_to_node(int cpu)
{
 if ((x86_cpu_to_node_map_early_ptr))
  return (x86_cpu_to_node_map_early_ptr)[cpu];

 return (*({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((&per_cpu__x86_cpu_to_node_map))); (typeof((&per_cpu__x86_cpu_to_node_map))) (__ptr + (((((_cpu_pda[cpu])->data_offset))))); }));
}


static inline cpumask_t *_node_to_cpumask_ptr(int node)
{
 return &node_to_cpumask_map[node];
}


static inline cpumask_t node_to_cpumask(int node)
{
 return node_to_cpumask_map[node];
}
extern int __node_distance(int, int);



static inline int node_to_first_cpu(int node)
{
 cpumask_t *mask = _node_to_cpumask_ptr(node);
 return __first_cpu(&(*mask));
}


extern cpumask_t cpu_coregroup_map(int cpu);
static inline void arch_fix_phys_package_id(int num, u32 slot)
{
}

struct pci_bus;
void set_pci_bus_resources_arch_default(struct pci_bus *b);







extern int get_mp_bus_to_node(int busnum);
extern void set_mp_bus_to_node(int busnum, int node);
void arch_update_cpu_topology(void);



struct memnode {
 int shift;
 unsigned int mapsize;
 s16 *map;
 s16 embedded_map[64 - 8];
} __attribute__((__aligned__((1 << (7)))));
extern struct memnode memnode;




extern struct pglist_data *node_data[];

static inline __attribute__((pure)) int phys_to_nid(unsigned long addr)
{
 unsigned nid;
 ;
 ;
 nid = memnode.map[addr >> memnode.shift];
 ;
 return nid;
}



extern struct pglist_data *first_online_pgdat(void);
extern struct pglist_data *next_online_pgdat(struct pglist_data *pgdat);
extern struct zone *next_zone(struct zone *zone);
static inline struct zone *zonelist_zone(struct zoneref *zoneref)
{
 return zoneref->zone;
}

static inline int zonelist_zone_idx(struct zoneref *zoneref)
{
 return zoneref->zone_idx;
}

static inline int zonelist_node_idx(struct zoneref *zoneref)
{


 return zoneref->zone->node;



}
struct zoneref *next_zones_zonelist(struct zoneref *z,
     enum zone_type highest_zoneidx,
     nodemask_t *nodes,
     struct zone **zone);
static inline struct zoneref *first_zones_zonelist(struct zonelist *zonelist,
     enum zone_type highest_zoneidx,
     nodemask_t *nodes,
     struct zone **zone)
{
 return next_zones_zonelist(zonelist->_zonerefs, highest_zoneidx, nodes,
        zone);
}
struct page;
struct mem_section {
 unsigned long section_mem_map;


 unsigned long *pageblock_flags;
};
extern struct mem_section *mem_section[((1UL << (44 - 27)) / (((1UL) << 12) / sizeof (struct mem_section)))];




static inline struct mem_section *__nr_to_section(unsigned long nr)
{
 if (!mem_section[((nr) / (((1UL) << 12) / sizeof (struct mem_section)))])
  return 0;
 return &mem_section[((nr) / (((1UL) << 12) / sizeof (struct mem_section)))][nr & ((((1UL) << 12) / sizeof (struct mem_section)) - 1)];
}
extern int __section_nr(struct mem_section* ms);
extern unsigned long usemap_size(void);
static inline struct page *__section_mem_map_addr(struct mem_section *section)
{
 unsigned long map = section->section_mem_map;
 map &= (~((1UL<<2)-1));
 return (struct page *)map;
}

static inline int present_section(struct mem_section *section)
{
 return (section && (section->section_mem_map & (1UL<<0)));
}

static inline int present_section_nr(unsigned long nr)
{
 return present_section(__nr_to_section(nr));
}

static inline int valid_section(struct mem_section *section)
{
 return (section && (section->section_mem_map & (1UL<<1)));
}

static inline int valid_section_nr(unsigned long nr)
{
 return valid_section(__nr_to_section(nr));
}

static inline struct mem_section *__pfn_to_section(unsigned long pfn)
{
 return __nr_to_section(((pfn) >> (27 - 12)));
}

static inline int pfn_valid(unsigned long pfn)
{
 if (((pfn) >> (27 - 12)) >= (1UL << (44 - 27)))
  return 0;
 return valid_section(__nr_to_section(((pfn) >> (27 - 12))));
}

static inline int pfn_present(unsigned long pfn)
{
 if (((pfn) >> (27 - 12)) >= (1UL << (44 - 27)))
  return 0;
 return present_section(__nr_to_section(((pfn) >> (27 - 12))));
}
void sparse_init(void);






bool early_pfn_in_nid(unsigned long pfn, int nid);
void memory_present(int nid, unsigned long start, unsigned long end);
unsigned long __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) node_memmap_size_bytes(int, unsigned long, unsigned long);



struct vm_area_struct;
static inline int allocflags_to_migratetype(gfp_t gfp_flags)
{
 ;

 if (__builtin_expect(!!(page_group_by_mobility_disabled), 0))
  return 0;


 return (((gfp_flags & (( gfp_t)0x100000u)) != 0) << 1) |
  ((gfp_flags & (( gfp_t)0x80000u)) != 0);
}

static inline enum zone_type gfp_zone(gfp_t flags)
{

 if (flags & (( gfp_t)0x01u))
  return ZONE_DMA;


 if (flags & (( gfp_t)0x04u))
  return ZONE_DMA32;

 if ((flags & ((( gfp_t)0x02u) | (( gfp_t)0x100000u))) ==
   ((( gfp_t)0x02u) | (( gfp_t)0x100000u)))
  return ZONE_MOVABLE;




 return ZONE_NORMAL;
}
static inline int gfp_zonelist(gfp_t flags)
{
 if (1 && __builtin_expect(!!(flags & (( gfp_t)0x40000u)), 0))
  return 1;

 return 0;
}
static inline struct zonelist *node_zonelist(int nid, gfp_t flags)
{
 return (node_data[nid])->node_zonelists + gfp_zonelist(flags);
}


static inline void arch_free_page(struct page *page, int order) { }


static inline void arch_alloc_page(struct page *page, int order) { }


struct page *
__alloc_pages_internal(gfp_t gfp_mask, unsigned int order,
         struct zonelist *zonelist, nodemask_t *nodemask);

static inline struct page *
__alloc_pages(gfp_t gfp_mask, unsigned int order,
  struct zonelist *zonelist)
{
 return __alloc_pages_internal(gfp_mask, order, zonelist, 0);
}

static inline struct page *
__alloc_pages_nodemask(gfp_t gfp_mask, unsigned int order,
  struct zonelist *zonelist, nodemask_t *nodemask)
{
 return __alloc_pages_internal(gfp_mask, order, zonelist, nodemask);
}


static inline struct page *alloc_pages_node(int nid, gfp_t gfp_mask,
      unsigned int order)
{
 if (__builtin_expect(!!(order >= 11), 0))
  return 0;


 if (nid < 0)
  nid = ({ typeof(_proxy_pda.nodenumber) ret__; switch (sizeof(_proxy_pda.nodenumber)) { case 2: asm("mov" "w %%gs:%c1,%0" : "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->nodenumber)), "m" (_proxy_pda.nodenumber)); break; case 4: asm("mov" "l %%gs:%c1,%0": "=r" (ret__): "i" (((size_t) &((struct x8664_pda *)0)->nodenumber)), "m" (_proxy_pda.nodenumber)); break; case 8: asm("mov" "q %%gs:%c1,%0": "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->nodenumber)), "m" (_proxy_pda.nodenumber)); break; default: __bad_pda_field(); } ret__; });

 return __alloc_pages(gfp_mask, order, node_zonelist(nid, gfp_mask));
}


extern struct page *alloc_pages_current(gfp_t gfp_mask, unsigned order);

static inline struct page *
alloc_pages(gfp_t gfp_mask, unsigned int order)
{
 if (__builtin_expect(!!(order >= 11), 0))
  return 0;

 return alloc_pages_current(gfp_mask, order);
}
extern struct page *alloc_page_vma(gfp_t gfp_mask,
   struct vm_area_struct *vma, unsigned long addr);







extern unsigned long __get_free_pages(gfp_t gfp_mask, unsigned int order);
extern unsigned long get_zeroed_page(gfp_t gfp_mask);

void *alloc_pages_exact(size_t size, gfp_t gfp_mask);
void free_pages_exact(void *virt, size_t size);







extern void __free_pages(struct page *page, unsigned int order);
extern void free_pages(unsigned long addr, unsigned int order);
extern void free_hot_page(struct page *page);
extern void free_cold_page(struct page *page);




void page_alloc_init(void);
void drain_zone_pages(struct zone *zone, struct per_cpu_pages *pcp);
void drain_all_pages(void);
void drain_local_pages(void *dummy);
void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) kmem_cache_init(void);
int slab_is_available(void);

struct kmem_cache *kmem_cache_create( char *, size_t, size_t,
   unsigned long,
   void (*)(void *));
void kmem_cache_destroy(struct kmem_cache *);
int kmem_cache_shrink(struct kmem_cache *);
void kmem_cache_free(struct kmem_cache *, void *);
unsigned int kmem_cache_size(struct kmem_cache *);
 char *kmem_cache_name(struct kmem_cache *);
int kmem_ptr_validate(struct kmem_cache *cachep, void *ptr);
void * __attribute__((warn_unused_result)) __krealloc( void *, size_t, gfp_t);
void * __attribute__((warn_unused_result)) krealloc( void *, size_t, gfp_t);
void kfree( void *);
size_t ksize( void *);











union ktime {
 s64 tv64;
};

typedef union ktime ktime_t;
static inline ktime_t ktime_set( long secs, unsigned long nsecs)
{

 if (__builtin_expect(!!(secs >= (((s64)~((u64)1 << 63)) / 1000000000L)), 0))

                return ({ ktime_t var; var.tv64 = ((s64)~((u64)1 << 63)); var; });





 return ({ ktime_t var; var.tv64 = (s64)secs * 1000000000L + (s64)nsecs; var; });



}
static inline ktime_t timespec_to_ktime(struct timespec ts)
{
 return ktime_set(ts.tv_sec, ts.tv_nsec);
}


static inline ktime_t timeval_to_ktime(struct timeval tv)
{
 return ktime_set(tv.tv_sec, tv.tv_usec * 1000L);
}
static inline int ktime_equal( ktime_t cmp1, ktime_t cmp2)
{
 return cmp1.tv64 == cmp2.tv64;
}

static inline s64 ktime_to_us( ktime_t kt)
{
 struct timeval tv = ns_to_timeval((kt).tv64);
 return (s64) tv.tv_sec * 1000000L + tv.tv_usec;
}

static inline s64 ktime_us_delta( ktime_t later, ktime_t earlier)
{
       return ktime_to_us(({ ({ ktime_t var; var.tv64 = (later).tv64 - (earlier).tv64; var; }); }));
}

static inline ktime_t ktime_add_us( ktime_t kt, u64 usec)
{
 return ({ ({ ktime_t var; var.tv64 = (kt).tv64 + (usec * 1000); var; }); });
}

static inline ktime_t ktime_sub_us( ktime_t kt, u64 usec)
{
 return ({ ({ ktime_t var; var.tv64 = (kt).tv64 - (usec * 1000); var; }); });
}

extern ktime_t ktime_add_safe( ktime_t lhs, ktime_t rhs);
extern void ktime_get_ts(struct timespec *ts);




static inline ktime_t ns_to_ktime(u64 ns)
{

        static ktime_t ktime_zero = { 0 };



 return ({ ({ ktime_t var; var.tv64 = (ktime_zero).tv64 + (ns); var; }); });
}







enum debug_obj_state {
 ODEBUG_STATE_NONE,
 ODEBUG_STATE_INIT,
 ODEBUG_STATE_INACTIVE,
 ODEBUG_STATE_ACTIVE,
 ODEBUG_STATE_DESTROYED,
 ODEBUG_STATE_NOTAVAILABLE,
 ODEBUG_STATE_MAX,
};

struct debug_obj_descr;
struct debug_obj {
 struct hlist_node node;
 enum debug_obj_state state;
 void *object;
 struct debug_obj_descr *descr;
};
struct debug_obj_descr {
 char *name;

 int (*fixup_init) (void *addr, enum debug_obj_state state);
 int (*fixup_activate) (void *addr, enum debug_obj_state state);
 int (*fixup_destroy) (void *addr, enum debug_obj_state state);
 int (*fixup_free) (void *addr, enum debug_obj_state state);
};
static inline void
debug_object_init (void *addr, struct debug_obj_descr *descr) { }
static inline void
debug_object_init_on_stack(void *addr, struct debug_obj_descr *descr) { }
static inline void
debug_object_activate (void *addr, struct debug_obj_descr *descr) { }
static inline void
debug_object_deactivate(void *addr, struct debug_obj_descr *descr) { }
static inline void
debug_object_destroy (void *addr, struct debug_obj_descr *descr) { }
static inline void
debug_object_free (void *addr, struct debug_obj_descr *descr) { }

static inline void debug_objects_early_init(void) { }
static inline void debug_objects_mem_init(void) { }





static inline void
debug_check_no_obj_freed( void *address, unsigned long size) { }

struct hrtimer_clock_base;
struct hrtimer_cpu_base;




enum hrtimer_mode {
 HRTIMER_MODE_ABS,
 HRTIMER_MODE_REL,
};




enum hrtimer_restart {
 HRTIMER_NORESTART,
 HRTIMER_RESTART,
};
enum hrtimer_cb_mode {
 HRTIMER_CB_SOFTIRQ,
 HRTIMER_CB_IRQSAFE,
 HRTIMER_CB_IRQSAFE_NO_RESTART,
 HRTIMER_CB_IRQSAFE_PERCPU,
 HRTIMER_CB_IRQSAFE_UNLOCKED,
};
struct hrtimer {
 struct rb_node node;
 ktime_t expires;
 enum hrtimer_restart (*function)(struct hrtimer *);
 struct hrtimer_clock_base *base;
 unsigned long state;
 enum hrtimer_cb_mode cb_mode;
 struct list_head cb_entry;

 void *start_site;
 char start_comm[16];
 int start_pid;

};
struct hrtimer_sleeper {
 struct hrtimer timer;
 struct task_struct *task;
};
struct hrtimer_clock_base {
 struct hrtimer_cpu_base *cpu_base;
 clockid_t index;
 struct rb_root active;
 struct rb_node *first;
 ktime_t resolution;
 ktime_t (*get_time)(void);
 ktime_t (*get_softirq_time)(void);
 ktime_t softirq_time;

 ktime_t offset;
 int (*reprogram)(struct hrtimer *t,
          struct hrtimer_clock_base *b,
          ktime_t n);

};
struct hrtimer_cpu_base {
 spinlock_t lock;
 struct hrtimer_clock_base clock_base[2];
 struct list_head cb_pending;

 ktime_t expires_next;
 int hres_active;
 unsigned long nr_events;

};


struct clock_event_device;

extern void clock_was_set(void);
extern void hres_timers_resume(void);
extern void hrtimer_interrupt(struct clock_event_device *dev);




static inline ktime_t hrtimer_cb_get_time(struct hrtimer *timer)
{
 return timer->base->get_time();
}

static inline int hrtimer_is_hres_active(struct hrtimer *timer)
{
 return timer->base->cpu_base->hres_active;
}
extern ktime_t ktime_get(void);
extern ktime_t ktime_get_real(void);




extern void hrtimer_init(struct hrtimer *timer, clockid_t which_clock,
    enum hrtimer_mode mode);







static inline void hrtimer_init_on_stack(struct hrtimer *timer,
      clockid_t which_clock,
      enum hrtimer_mode mode)
{
 hrtimer_init(timer, which_clock, mode);
}
static inline void destroy_hrtimer_on_stack(struct hrtimer *timer) { }



extern int hrtimer_start(struct hrtimer *timer, ktime_t tim,
    enum hrtimer_mode mode);
extern int hrtimer_cancel(struct hrtimer *timer);
extern int hrtimer_try_to_cancel(struct hrtimer *timer);

static inline int hrtimer_restart(struct hrtimer *timer)
{
 return hrtimer_start(timer, timer->expires, HRTIMER_MODE_ABS);
}


extern ktime_t hrtimer_get_remaining( struct hrtimer *timer);
extern int hrtimer_get_res( clockid_t which_clock, struct timespec *tp);

extern ktime_t hrtimer_get_next_event(void);





static inline int hrtimer_active( struct hrtimer *timer)
{
 return timer->state != 0x00;
}




static inline int hrtimer_is_queued(struct hrtimer *timer)
{
 return timer->state &
  (0x01 | 0x04);
}





static inline int hrtimer_callback_running(struct hrtimer *timer)
{
 return timer->state & 0x02;
}


extern u64
hrtimer_forward(struct hrtimer *timer, ktime_t now, ktime_t interval);


static inline u64 hrtimer_forward_now(struct hrtimer *timer,
          ktime_t interval)
{
 return hrtimer_forward(timer, timer->base->get_time(), interval);
}


extern long hrtimer_nanosleep(struct timespec *rqtp,
         struct timespec *rmtp,
         enum hrtimer_mode mode,
         clockid_t clockid);
extern long hrtimer_nanosleep_restart(struct restart_block *restart_block);

extern void hrtimer_init_sleeper(struct hrtimer_sleeper *sl,
     struct task_struct *tsk);


extern void hrtimer_run_queues(void);
extern void hrtimer_run_pending(void);


extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) hrtimers_init(void);
extern void sysrq_timer_list_show(void);






extern void timer_stats_update_stats(void *timer, pid_t pid, void *startf,
         void *timerf, char *comm,
         unsigned int timer_flag);

static inline void timer_stats_account_hrtimer(struct hrtimer *timer)
{
 timer_stats_update_stats(timer, timer->start_pid, timer->start_site,
     timer->function, timer->start_comm, 0);
}

extern void __timer_stats_hrtimer_set_start_info(struct hrtimer *timer,
       void *addr);

static inline void timer_stats_hrtimer_set_start_info(struct hrtimer *timer)
{
 __timer_stats_hrtimer_set_start_info(timer, __builtin_return_address(0));
}

static inline void timer_stats_hrtimer_clear_start_info(struct hrtimer *timer)
{
 timer->start_site = 0;
}

struct tvec_base;

struct timer_list {
 struct list_head entry;
 unsigned long expires;

 void (*function)(unsigned long);
 unsigned long data;

 struct tvec_base *base;

 void *start_site;
 char start_comm[16];
 int start_pid;

};

extern struct tvec_base boot_tvec_bases;
void init_timer(struct timer_list *timer);
void init_timer_deferrable(struct timer_list *timer);





static inline void destroy_timer_on_stack(struct timer_list *timer) { }
static inline void init_timer_on_stack(struct timer_list *timer)
{
 init_timer(timer);
}


static inline void setup_timer(struct timer_list * timer,
    void (*function)(unsigned long),
    unsigned long data)
{
 timer->function = function;
 timer->data = data;
 init_timer(timer);
}

static inline void setup_timer_on_stack(struct timer_list *timer,
     void (*function)(unsigned long),
     unsigned long data)
{
 timer->function = function;
 timer->data = data;
 init_timer_on_stack(timer);
}
static inline int timer_pending( struct timer_list * timer)
{
 return timer->entry.next != 0;
}

extern void add_timer_on(struct timer_list *timer, int cpu);
extern int del_timer(struct timer_list * timer);
extern int __mod_timer(struct timer_list *timer, unsigned long expires);
extern int mod_timer(struct timer_list *timer, unsigned long expires);
extern unsigned long next_timer_interrupt(void);





extern unsigned long get_next_timer_interrupt(unsigned long now);
extern void init_timer_stats(void);

extern void timer_stats_update_stats(void *timer, pid_t pid, void *startf,
         void *timerf, char *comm,
         unsigned int timer_flag);

extern void __timer_stats_timer_set_start_info(struct timer_list *timer,
            void *addr);

static inline void timer_stats_timer_set_start_info(struct timer_list *timer)
{
 __timer_stats_timer_set_start_info(timer, __builtin_return_address(0));
}

static inline void timer_stats_timer_clear_start_info(struct timer_list *timer)
{
 timer->start_site = 0;
}
static inline void add_timer(struct timer_list *timer)
{
 ;
 __mod_timer(timer, timer->expires);
}


  extern int try_to_del_timer_sync(struct timer_list *timer);
  extern int del_timer_sync(struct timer_list *timer);







extern void init_timers(void);
extern void run_local_timers(void);
struct hrtimer;
extern enum hrtimer_restart it_real_fn(struct hrtimer *);

unsigned long __round_jiffies(unsigned long j, int cpu);
unsigned long __round_jiffies_relative(unsigned long j, int cpu);
unsigned long round_jiffies(unsigned long j);
unsigned long round_jiffies_relative(unsigned long j);




struct workqueue_struct;

struct work_struct;
typedef void (*work_func_t)(struct work_struct *work);







struct work_struct {
 atomic_long_t data;



 struct list_head entry;
 work_func_t func;



};



struct delayed_work {
 struct work_struct work;
 struct timer_list timer;
};

struct execute_work {
 struct work_struct work;
};
extern struct workqueue_struct *
__create_workqueue_key( char *name, int singlethread,
         int freezeable, struct lock_class_key *key,
         char *lock_name);
extern void destroy_workqueue(struct workqueue_struct *wq);

extern int queue_work(struct workqueue_struct *wq, struct work_struct *work);
extern int queue_work_on(int cpu, struct workqueue_struct *wq,
   struct work_struct *work);
extern int queue_delayed_work(struct workqueue_struct *wq,
   struct delayed_work *work, unsigned long delay);
extern int queue_delayed_work_on(int cpu, struct workqueue_struct *wq,
   struct delayed_work *work, unsigned long delay);

extern void flush_workqueue(struct workqueue_struct *wq);
extern void flush_scheduled_work(void);

extern int schedule_work(struct work_struct *work);
extern int schedule_work_on(int cpu, struct work_struct *work);
extern int schedule_delayed_work(struct delayed_work *work, unsigned long delay);
extern int schedule_delayed_work_on(int cpu, struct delayed_work *work,
     unsigned long delay);
extern int schedule_on_each_cpu(work_func_t func);
extern int current_is_keventd(void);
extern int keventd_up(void);

extern void init_workqueues(void);
int execute_in_process_context(work_func_t fn, struct execute_work *);

extern int flush_work(struct work_struct *work);

extern int cancel_work_sync(struct work_struct *work);







static inline int cancel_delayed_work(struct delayed_work *work)
{
 int ret;

 ret = del_timer_sync(&work->timer);
 if (ret)
  clear_bit(0, ((unsigned long *)(&(&work->work)->data)));
 return ret;
}

extern int cancel_delayed_work_sync(struct delayed_work *work);


static inline
void cancel_rearming_delayed_workqueue(struct workqueue_struct *wq,
     struct delayed_work *work)
{
 cancel_delayed_work_sync(work);
}


static inline
void cancel_rearming_delayed_work(struct delayed_work *work)
{
 cancel_delayed_work_sync(work);
}

struct kobject;
struct module;





struct attribute {
 char *name;
 struct module *owner;
 mode_t mode;
};

struct attribute_group {
 char *name;
 mode_t (*is_visible)(struct kobject *,
           struct attribute *, int);
 struct attribute **attrs;
};
struct vm_area_struct;

struct bin_attribute {
 struct attribute attr;
 size_t size;
 void *_private;
 ssize_t (*read)(struct kobject *, struct bin_attribute *,
   char *, loff_t, size_t);
 ssize_t (*write)(struct kobject *, struct bin_attribute *,
    char *, loff_t, size_t);
 int (*mmap)(struct kobject *, struct bin_attribute *attr,
      struct vm_area_struct *vma);
};

struct sysfs_ops {
 ssize_t (*show)(struct kobject *, struct attribute *,char *);
 ssize_t (*store)(struct kobject *,struct attribute *, char *, size_t);
};



int sysfs_schedule_callback(struct kobject *kobj, void (*func)(void *),
       void *data, struct module *owner);

int __attribute__((warn_unused_result)) sysfs_create_dir(struct kobject *kobj);
void sysfs_remove_dir(struct kobject *kobj);
int __attribute__((warn_unused_result)) sysfs_rename_dir(struct kobject *kobj, char *new_name);
int __attribute__((warn_unused_result)) sysfs_move_dir(struct kobject *kobj,
    struct kobject *new_parent_kobj);

int __attribute__((warn_unused_result)) sysfs_create_file(struct kobject *kobj,
       struct attribute *attr);
int __attribute__((warn_unused_result)) sysfs_chmod_file(struct kobject *kobj, struct attribute *attr,
      mode_t mode);
void sysfs_remove_file(struct kobject *kobj, struct attribute *attr);

int __attribute__((warn_unused_result)) sysfs_create_bin_file(struct kobject *kobj,
           struct bin_attribute *attr);
void sysfs_remove_bin_file(struct kobject *kobj, struct bin_attribute *attr);

int __attribute__((warn_unused_result)) sysfs_create_link(struct kobject *kobj, struct kobject *target,
       char *name);
int __attribute__((warn_unused_result)) sysfs_create_link_nowarn(struct kobject *kobj,
       struct kobject *target,
       char *name);
void sysfs_remove_link(struct kobject *kobj, char *name);

int __attribute__((warn_unused_result)) sysfs_create_group(struct kobject *kobj,
        struct attribute_group *grp);
int sysfs_update_group(struct kobject *kobj,
         struct attribute_group *grp);
void sysfs_remove_group(struct kobject *kobj,
   struct attribute_group *grp);
int sysfs_add_file_to_group(struct kobject *kobj,
   struct attribute *attr, char *group);
void sysfs_remove_file_from_group(struct kobject *kobj,
   struct attribute *attr, char *group);

void sysfs_notify(struct kobject *kobj, char *dir, char *attr);

extern int __attribute__((warn_unused_result)) sysfs_init(void);











extern char uevent_helper[];


extern u64 uevent_seqnum;
enum kobject_action {
 KOBJ_ADD,
 KOBJ_REMOVE,
 KOBJ_CHANGE,
 KOBJ_MOVE,
 KOBJ_ONLINE,
 KOBJ_OFFLINE,
 KOBJ_MAX
};

struct kobject {
 char *name;
 struct list_head entry;
 struct kobject *parent;
 struct kset *kset;
 struct kobj_type *ktype;
 struct sysfs_dirent *sd;
 struct kref kref;
 unsigned int state_initialized:1;
 unsigned int state_in_sysfs:1;
 unsigned int state_add_uevent_sent:1;
 unsigned int state_remove_uevent_sent:1;
};

extern int kobject_set_name(struct kobject *kobj, char *name, ...)
       __attribute__((format(printf, 2, 3)));

static inline char *kobject_name( struct kobject *kobj)
{
 return kobj->name;
}

extern void kobject_init(struct kobject *kobj, struct kobj_type *ktype);
extern int __attribute__((warn_unused_result)) kobject_add(struct kobject *kobj,
        struct kobject *parent,
        char *fmt, ...);
extern int __attribute__((warn_unused_result)) kobject_init_and_add(struct kobject *kobj,
          struct kobj_type *ktype,
          struct kobject *parent,
          char *fmt, ...);

extern void kobject_del(struct kobject *kobj);

extern struct kobject * __attribute__((warn_unused_result)) kobject_create(void);
extern struct kobject * __attribute__((warn_unused_result)) kobject_create_and_add( char *name,
      struct kobject *parent);

extern int __attribute__((warn_unused_result)) kobject_rename(struct kobject *, char *new_name);
extern int __attribute__((warn_unused_result)) kobject_move(struct kobject *, struct kobject *);

extern struct kobject *kobject_get(struct kobject *kobj);
extern void kobject_put(struct kobject *kobj);

extern char *kobject_get_path(struct kobject *kobj, gfp_t flag);

struct kobj_type {
 void (*release)(struct kobject *kobj);
 struct sysfs_ops *sysfs_ops;
 struct attribute **default_attrs;
};

struct kobj_uevent_env {
 char *envp[32];
 int envp_idx;
 char buf[2048];
 int buflen;
};

struct kset_uevent_ops {
 int (*filter)(struct kset *kset, struct kobject *kobj);
 char *(*name)(struct kset *kset, struct kobject *kobj);
 int (*uevent)(struct kset *kset, struct kobject *kobj,
        struct kobj_uevent_env *env);
};

struct kobj_attribute {
 struct attribute attr;
 ssize_t (*show)(struct kobject *kobj, struct kobj_attribute *attr,
   char *buf);
 ssize_t (*store)(struct kobject *kobj, struct kobj_attribute *attr,
    char *buf, size_t count);
};

extern struct sysfs_ops kobj_sysfs_ops;
struct kset {
 struct list_head list;
 spinlock_t list_lock;
 struct kobject kobj;
 struct kset_uevent_ops *uevent_ops;
};

extern void kset_init(struct kset *kset);
extern int __attribute__((warn_unused_result)) kset_register(struct kset *kset);
extern void kset_unregister(struct kset *kset);
extern struct kset * __attribute__((warn_unused_result)) kset_create_and_add( char *name,
      struct kset_uevent_ops *u,
      struct kobject *parent_kobj);

static inline struct kset *to_kset(struct kobject *kobj)
{
 return kobj ? ({ typeof( ((struct kset *)0)->kobj ) *__mptr = (kobj); (struct kset *)( (char *)__mptr - ((size_t) &((struct kset *)0)->kobj) );}) : 0;
}

static inline struct kset *kset_get(struct kset *k)
{
 return k ? to_kset(kobject_get(&k->kobj)) : 0;
}

static inline void kset_put(struct kset *k)
{
 kobject_put(&k->kobj);
}

static inline struct kobj_type *get_ktype(struct kobject *kobj)
{
 return kobj->ktype;
}

extern struct kobject *kset_find_obj(struct kset *, char *);


extern struct kobject *kernel_kobj;

extern struct kobject *mm_kobj;

extern struct kobject *hypervisor_kobj;

extern struct kobject *power_kobj;

extern struct kobject *firmware_kobj;


int kobject_uevent(struct kobject *kobj, enum kobject_action action);
int kobject_uevent_env(struct kobject *kobj, enum kobject_action action,
   char *envp[]);

int add_uevent_var(struct kobj_uevent_env *env, char *format, ...)
 __attribute__((format (printf, 2, 3)));

int kobject_action_type( char *buf, size_t count,
   enum kobject_action *type);

enum stat_item {
 ALLOC_FASTPATH,
 ALLOC_SLOWPATH,
 FREE_FASTPATH,
 FREE_SLOWPATH,
 FREE_FROZEN,
 FREE_ADD_PARTIAL,
 FREE_REMOVE_PARTIAL,
 ALLOC_FROM_PARTIAL,
 ALLOC_SLAB,
 ALLOC_REFILL,
 FREE_SLAB,
 CPUSLAB_FLUSH,
 DEACTIVATE_FULL,
 DEACTIVATE_EMPTY,
 DEACTIVATE_TO_HEAD,
 DEACTIVATE_TO_TAIL,
 DEACTIVATE_REMOTE_FREES,
 ORDER_FALLBACK,
 NR_SLUB_STAT_ITEMS };

struct kmem_cache_cpu {
 void **freelist;
 struct page *page;
 int node;
 unsigned int offset;
 unsigned int objsize;



};

struct kmem_cache_node {
 spinlock_t list_lock;
 unsigned long nr_partial;
 unsigned long min_partial;
 struct list_head partial;

 atomic_long_t nr_slabs;
 atomic_long_t total_objects;
 struct list_head full;

};






struct kmem_cache_order_objects {
 unsigned long x;
};




struct kmem_cache {

 unsigned long flags;
 int size;
 int objsize;
 int offset;
 struct kmem_cache_order_objects oo;





 struct kmem_cache_node local_node;


 struct kmem_cache_order_objects max;
 struct kmem_cache_order_objects min;
 gfp_t allocflags;
 int refcount;
 void (*ctor)(void *);
 int inuse;
 int align;
 char *name;
 struct list_head list;

 struct kobject kobj;






 int remote_node_defrag_ratio;
 struct kmem_cache_node *node[(1 << 9)];


 struct kmem_cache_cpu *cpu_slab[64];



};
extern struct kmem_cache kmalloc_caches[12 + 1];





static inline __attribute__((always_inline)) int kmalloc_index(size_t size)
{
 if (!size)
  return 0;

 if (size <= 8)
  return ( __builtin_constant_p(8) ? ( (8) < 1 ? ____ilog2_NaN() : (8) & (1ULL << 63) ? 63 : (8) & (1ULL << 62) ? 62 : (8) & (1ULL << 61) ? 61 : (8) & (1ULL << 60) ? 60 : (8) & (1ULL << 59) ? 59 : (8) & (1ULL << 58) ? 58 : (8) & (1ULL << 57) ? 57 : (8) & (1ULL << 56) ? 56 : (8) & (1ULL << 55) ? 55 : (8) & (1ULL << 54) ? 54 : (8) & (1ULL << 53) ? 53 : (8) & (1ULL << 52) ? 52 : (8) & (1ULL << 51) ? 51 : (8) & (1ULL << 50) ? 50 : (8) & (1ULL << 49) ? 49 : (8) & (1ULL << 48) ? 48 : (8) & (1ULL << 47) ? 47 : (8) & (1ULL << 46) ? 46 : (8) & (1ULL << 45) ? 45 : (8) & (1ULL << 44) ? 44 : (8) & (1ULL << 43) ? 43 : (8) & (1ULL << 42) ? 42 : (8) & (1ULL << 41) ? 41 : (8) & (1ULL << 40) ? 40 : (8) & (1ULL << 39) ? 39 : (8) & (1ULL << 38) ? 38 : (8) & (1ULL << 37) ? 37 : (8) & (1ULL << 36) ? 36 : (8) & (1ULL << 35) ? 35 : (8) & (1ULL << 34) ? 34 : (8) & (1ULL << 33) ? 33 : (8) & (1ULL << 32) ? 32 : (8) & (1ULL << 31) ? 31 : (8) & (1ULL << 30) ? 30 : (8) & (1ULL << 29) ? 29 : (8) & (1ULL << 28) ? 28 : (8) & (1ULL << 27) ? 27 : (8) & (1ULL << 26) ? 26 : (8) & (1ULL << 25) ? 25 : (8) & (1ULL << 24) ? 24 : (8) & (1ULL << 23) ? 23 : (8) & (1ULL << 22) ? 22 : (8) & (1ULL << 21) ? 21 : (8) & (1ULL << 20) ? 20 : (8) & (1ULL << 19) ? 19 : (8) & (1ULL << 18) ? 18 : (8) & (1ULL << 17) ? 17 : (8) & (1ULL << 16) ? 16 : (8) & (1ULL << 15) ? 15 : (8) & (1ULL << 14) ? 14 : (8) & (1ULL << 13) ? 13 : (8) & (1ULL << 12) ? 12 : (8) & (1ULL << 11) ? 11 : (8) & (1ULL << 10) ? 10 : (8) & (1ULL << 9) ? 9 : (8) & (1ULL << 8) ? 8 : (8) & (1ULL << 7) ? 7 : (8) & (1ULL << 6) ? 6 : (8) & (1ULL << 5) ? 5 : (8) & (1ULL << 4) ? 4 : (8) & (1ULL << 3) ? 3 : (8) & (1ULL << 2) ? 2 : (8) & (1ULL << 1) ? 1 : (8) & (1ULL << 0) ? 0 : ____ilog2_NaN() ) : (sizeof(8) <= 4) ? __ilog2_u32(8) : __ilog2_u64(8) );


 if (size > 64 && size <= 96)
  return 1;
 if (size > 128 && size <= 192)
  return 2;

 if (size <= 8) return 3;
 if (size <= 16) return 4;
 if (size <= 32) return 5;
 if (size <= 64) return 6;
 if (size <= 128) return 7;
 if (size <= 256) return 8;
 if (size <= 512) return 9;
 if (size <= 1024) return 10;
 if (size <= 2 * 1024) return 11;
 if (size <= 4 * 1024) return 12;




 if (size <= 8 * 1024) return 13;
 if (size <= 16 * 1024) return 14;
 if (size <= 32 * 1024) return 15;
 if (size <= 64 * 1024) return 16;
 if (size <= 128 * 1024) return 17;
 if (size <= 256 * 1024) return 18;
 if (size <= 512 * 1024) return 19;
 if (size <= 1024 * 1024) return 20;
 if (size <= 2 * 1024 * 1024) return 21;
 return -1;
}







static inline __attribute__((always_inline)) struct kmem_cache *kmalloc_slab(size_t size)
{
 int index = kmalloc_index(size);

 if (index == 0)
  return 0;

 return &kmalloc_caches[index];
}
void *kmem_cache_alloc(struct kmem_cache *, gfp_t);
void *__kmalloc(size_t size, gfp_t flags);

static inline __attribute__((always_inline)) void *kmalloc_large(size_t size, gfp_t flags)
{
 return (void *)__get_free_pages(flags | (( gfp_t)0x4000u), get_order(size));
}


extern void *kmalloc(size_t size, gfp_t flags);
void *__kmalloc_node(size_t size, gfp_t flags, int node);
void *kmem_cache_alloc_node(struct kmem_cache *, gfp_t flags, int node);

static inline __attribute__((always_inline)) void *kmalloc_node(size_t size, gfp_t flags, int node)
{
 if (__builtin_constant_p(size) &&
  size <= ((1UL) << 12) && !(flags & (( gfp_t)0x01u))) {
   struct kmem_cache *s = kmalloc_slab(size);

  if (!s)
   return ((void *)16);

  return kmem_cache_alloc_node(s, flags, node);
 }
 return __kmalloc_node(size, flags, node);
}
static inline void *kcalloc(size_t n, size_t size, gfp_t flags)
{
 if (size != 0 && n > (~0UL) / size)
  return 0;
 return __kmalloc(n * size, flags | (( gfp_t)0x8000u));
}
extern void *__kmalloc_track_caller(size_t, gfp_t, void*);
extern void *__kmalloc_node_track_caller(size_t, gfp_t, int, void *);
static inline void *kmem_cache_zalloc(struct kmem_cache *k, gfp_t flags)
{
 return kmem_cache_alloc(k, flags | (( gfp_t)0x8000u));
}






static inline void *kzalloc(size_t size, gfp_t flags)
{
 return kmalloc(size, flags | (( gfp_t)0x8000u));
}







static inline void *kzalloc_node(size_t size, gfp_t flags, int node)
{
 return kmalloc_node(size, flags | (( gfp_t)0x8000u), node);
}


extern struct seq_operations slabinfo_op;
ssize_t slabinfo_write(struct file *, char *, size_t, loff_t *);
struct percpu_data {
 void *ptrs[1];
};
extern void *__percpu_alloc_mask(size_t size, gfp_t gfp, cpumask_t *mask);
extern void percpu_free(void *__pdata);
struct rcu_head {
 struct rcu_head *next;
 void (*func)(struct rcu_head *head);
};


struct rcu_ctrlblk {
 long cur;
 long completed;
 int next_pending;

 int signaled;

 spinlock_t lock __attribute__((__aligned__(1 << ((7)))));
 cpumask_t cpumask;

} __attribute__((__aligned__(1 << ((7)))));


static inline int rcu_batch_before(long a, long b)
{
 return (a - b) < 0;
}


static inline int rcu_batch_after(long a, long b)
{
 return (a - b) > 0;
}






struct rcu_data {

 long quiescbatch;
 int passed_quiesc;
 int qs_pending;


 long batch;
 struct rcu_head *nxtlist;
 struct rcu_head **nxttail;
 long qlen;
 struct rcu_head *curlist;
 struct rcu_head **curtail;
 struct rcu_head *donelist;
 struct rcu_head **donetail;
 long blimit;
 int cpu;
 struct rcu_head barrier;
};

extern __typeof__(struct rcu_data) per_cpu__rcu_data;
extern __typeof__(struct rcu_data) per_cpu__rcu_bh_data;







static inline void rcu_qsctr_inc(int cpu)
{
 struct rcu_data *rdp = &(*({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((&per_cpu__rcu_data))); (typeof((&per_cpu__rcu_data))) (__ptr + (((((_cpu_pda[cpu])->data_offset))))); }));
 rdp->passed_quiesc = 1;
}
static inline void rcu_bh_qsctr_inc(int cpu)
{
 struct rcu_data *rdp = &(*({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((&per_cpu__rcu_bh_data))); (typeof((&per_cpu__rcu_bh_data))) (__ptr + (((((_cpu_pda[cpu])->data_offset))))); }));
 rdp->passed_quiesc = 1;
}

extern int rcu_pending(int cpu);
extern int rcu_needs_cpu(int cpu);
extern void __rcu_init(void);

extern void rcu_check_callbacks(int cpu, int user);
extern void rcu_restart_cpu(int cpu);

extern long rcu_batches_completed(void);
extern long rcu_batches_completed_bh(void);
struct rcu_synchronize {
 struct rcu_head head;
 struct completion completion;
};

extern void wakeme_after_rcu(struct rcu_head *head);
extern void call_rcu(struct rcu_head *head,
         void (*func)(struct rcu_head *head));
extern void call_rcu_bh(struct rcu_head *head,
   void (*func)(struct rcu_head *head));


extern void synchronize_rcu(void);
extern void rcu_barrier(void);
extern void rcu_barrier_bh(void);
extern void rcu_barrier_sched(void);


extern void rcu_init(void);
extern int rcu_needs_cpu(int cpu);

struct task_struct;


struct sem {
 int semval;
 int sempid;
};


struct sem_array {
 struct kern_ipc_perm sem_perm;
 time_t sem_otime;
 time_t sem_ctime;
 struct sem *sem_base;
 struct list_head sem_pending;
 struct list_head list_id;
 unsigned long sem_nsems;
};


struct sem_queue {
 struct list_head list;
 struct task_struct *sleeper;
 struct sem_undo *undo;
 int pid;
 int status;
 struct sembuf *sops;
 int nsops;
 int alter;
};




struct sem_undo {
 struct list_head list_proc;

 struct rcu_head rcu;
 struct sem_undo_list *ulp;
 struct list_head list_id;
 int semid;
 short * semadj;
};




struct sem_undo_list {
 atomic_t refcnt;
 spinlock_t lock;
 struct list_head list_proc;
};

struct sysv_sem {
 struct sem_undo_list *undo_list;
};



extern int copy_semundo(unsigned long clone_flags, struct task_struct *tsk);
extern void exit_sem(struct task_struct *tsk);



struct siginfo;
typedef unsigned long old_sigset_t;

typedef struct {
 unsigned long sig[(64 / 64)];
} sigset_t;
typedef void __signalfn_t(int);
typedef __signalfn_t *__sighandler_t;

typedef void __restorefn_t(void);
typedef __restorefn_t *__sigrestore_t;
struct sigaction {
 __sighandler_t sa_handler;
 unsigned long sa_flags;
 __sigrestore_t sa_restorer;
 sigset_t sa_mask;
};

struct k_sigaction {
 struct sigaction sa;
};



typedef struct sigaltstack {
 void *ss_sp;
 int ss_flags;
 size_t ss_size;
} stack_t;













typedef union sigval {
 int sival_int;
 void *sival_ptr;
} sigval_t;
typedef struct siginfo {
 int si_signo;
 int si_errno;
 int si_code;

 union {
  int _pad[((128 - (4 * sizeof(int))) / sizeof(int))];


  struct {
   pid_t _pid;
   uid_t _uid;
  } _kill;


  struct {
   timer_t _tid;
   int _overrun;
   char _pad[sizeof( uid_t) - sizeof(int)];
   sigval_t _sigval;
   int _sys_private;
  } _timer;


  struct {
   pid_t _pid;
   uid_t _uid;
   sigval_t _sigval;
  } _rt;


  struct {
   pid_t _pid;
   uid_t _uid;
   int _status;
   clock_t _utime;
   clock_t _stime;
  } _sigchld;


  struct {
   void *_addr;



  } _sigfault;


  struct {
   long _band;
   int _fd;
  } _sigpoll;
 } _sifields;
} siginfo_t;
typedef struct sigevent {
 sigval_t sigev_value;
 int sigev_signo;
 int sigev_notify;
 union {
  int _pad[((64 - (sizeof(int) * 2 + sizeof(sigval_t))) / sizeof(int))];
   int _tid;

  struct {
   void (*_function)(sigval_t);
   void *_attribute;
  } _sigev_thread;
 } _sigev_un;
} sigevent_t;







struct siginfo;
void do_schedule_next_timer(struct siginfo *info);





static inline void copy_siginfo(struct siginfo *to, struct siginfo *from)
{
 if (from->si_code < 0)
  memcpy(to, from, sizeof(*to));
 else

  memcpy(to, from, (4 * sizeof(int)) + sizeof(from->_sifields._sigchld));
}



extern int copy_siginfo_to_user(struct siginfo *to, struct siginfo *from);
struct sigqueue {
 struct list_head list;
 int flags;
 siginfo_t info;
 struct user_struct *user;
};




struct sigpending {
 struct list_head list;
 sigset_t signal;
};
static inline void sigaddset(sigset_t *set, int _sig)
{
 unsigned long sig = _sig - 1;
 if ((64 / 64) == 1)
  set->sig[0] |= 1UL << sig;
 else
  set->sig[sig / 64] |= 1UL << (sig % 64);
}

static inline void sigdelset(sigset_t *set, int _sig)
{
 unsigned long sig = _sig - 1;
 if ((64 / 64) == 1)
  set->sig[0] &= ~(1UL << sig);
 else
  set->sig[sig / 64] &= ~(1UL << (sig % 64));
}

static inline int sigismember(sigset_t *set, int _sig)
{
 unsigned long sig = _sig - 1;
 if ((64 / 64) == 1)
  return 1 & (set->sig[0] >> sig);
 else
  return 1 & (set->sig[sig / 64] >> (sig % 64));
}

static inline int sigfindinword(unsigned long word)
{
 return ffz(~word);
}



static inline int sigisemptyset(sigset_t *set)
{
 extern void _NSIG_WORDS_is_unsupported_size(void);
 switch ((64 / 64)) {
 case 4:
  return (set->sig[3] | set->sig[2] |
   set->sig[1] | set->sig[0]) == 0;
 case 2:
  return (set->sig[1] | set->sig[0]) == 0;
 case 1:
  return set->sig[0] == 0;
 default:
  _NSIG_WORDS_is_unsupported_size();
  return 0;
 }
}
static inline void sigorsets(sigset_t *r, sigset_t *a, sigset_t *b) { extern void _NSIG_WORDS_is_unsupported_size(void); unsigned long a0, a1, a2, a3, b0, b1, b2, b3; switch ((64 / 64)) { case 4: a3 = a->sig[3]; a2 = a->sig[2]; b3 = b->sig[3]; b2 = b->sig[2]; r->sig[3] = ((a3) | (b3)); r->sig[2] = ((a2) | (b2)); case 2: a1 = a->sig[1]; b1 = b->sig[1]; r->sig[1] = ((a1) | (b1)); case 1: a0 = a->sig[0]; b0 = b->sig[0]; r->sig[0] = ((a0) | (b0)); break; default: _NSIG_WORDS_is_unsupported_size(); } }


static inline void sigandsets(sigset_t *r, sigset_t *a, sigset_t *b) { extern void _NSIG_WORDS_is_unsupported_size(void); unsigned long a0, a1, a2, a3, b0, b1, b2, b3; switch ((64 / 64)) { case 4: a3 = a->sig[3]; a2 = a->sig[2]; b3 = b->sig[3]; b2 = b->sig[2]; r->sig[3] = ((a3) & (b3)); r->sig[2] = ((a2) & (b2)); case 2: a1 = a->sig[1]; b1 = b->sig[1]; r->sig[1] = ((a1) & (b1)); case 1: a0 = a->sig[0]; b0 = b->sig[0]; r->sig[0] = ((a0) & (b0)); break; default: _NSIG_WORDS_is_unsupported_size(); } }


static inline void signandsets(sigset_t *r, sigset_t *a, sigset_t *b) { extern void _NSIG_WORDS_is_unsupported_size(void); unsigned long a0, a1, a2, a3, b0, b1, b2, b3; switch ((64 / 64)) { case 4: a3 = a->sig[3]; a2 = a->sig[2]; b3 = b->sig[3]; b2 = b->sig[2]; r->sig[3] = ((a3) & ~(b3)); r->sig[2] = ((a2) & ~(b2)); case 2: a1 = a->sig[1]; b1 = b->sig[1]; r->sig[1] = ((a1) & ~(b1)); case 1: a0 = a->sig[0]; b0 = b->sig[0]; r->sig[0] = ((a0) & ~(b0)); break; default: _NSIG_WORDS_is_unsupported_size(); } }
static inline void signotset(sigset_t *set) { extern void _NSIG_WORDS_is_unsupported_size(void); switch ((64 / 64)) { case 4: set->sig[3] = (~(set->sig[3])); set->sig[2] = (~(set->sig[2])); case 2: set->sig[1] = (~(set->sig[1])); case 1: set->sig[0] = (~(set->sig[0])); break; default: _NSIG_WORDS_is_unsupported_size(); } }




static inline void sigemptyset(sigset_t *set)
{
 switch ((64 / 64)) {
 default:
  memset(set, 0, sizeof(sigset_t));
  break;
 case 2: set->sig[1] = 0;
 case 1: set->sig[0] = 0;
  break;
 }
}

static inline void sigfillset(sigset_t *set)
{
 switch ((64 / 64)) {
 default:
  memset(set, -1, sizeof(sigset_t));
  break;
 case 2: set->sig[1] = -1;
 case 1: set->sig[0] = -1;
  break;
 }
}



static inline void sigaddsetmask(sigset_t *set, unsigned long mask)
{
 set->sig[0] |= mask;
}

static inline void sigdelsetmask(sigset_t *set, unsigned long mask)
{
 set->sig[0] &= ~mask;
}

static inline int sigtestsetmask(sigset_t *set, unsigned long mask)
{
 return (set->sig[0] & mask) != 0;
}

static inline void siginitset(sigset_t *set, unsigned long mask)
{
 set->sig[0] = mask;
 switch ((64 / 64)) {
 default:
  memset(&set->sig[1], 0, sizeof(long)*((64 / 64)-1));
  break;
 case 2: set->sig[1] = 0;
 case 1: ;
 }
}

static inline void siginitsetinv(sigset_t *set, unsigned long mask)
{
 set->sig[0] = ~mask;
 switch ((64 / 64)) {
 default:
  memset(&set->sig[1], -1, sizeof(long)*((64 / 64)-1));
  break;
 case 2: set->sig[1] = -1;
 case 1: ;
 }
}



static inline void init_sigpending(struct sigpending *sig)
{
 sigemptyset(&sig->signal);
 INIT_LIST_HEAD(&sig->list);
}

extern void flush_sigqueue(struct sigpending *queue);


static inline int valid_signal(unsigned long sig)
{
 return sig <= 64 ? 1 : 0;
}

extern int next_signal(struct sigpending *pending, sigset_t *mask);
extern int group_send_sig_info(int sig, struct siginfo *info, struct task_struct *p);
extern int __group_send_sig_info(int, struct siginfo *, struct task_struct *);
extern long do_sigpending(void *, unsigned long);
extern int sigprocmask(int, sigset_t *, sigset_t *);
extern int show_unhandled_signals;

struct pt_regs;
extern int get_signal_to_deliver(siginfo_t *info, struct k_sigaction *return_ka, struct pt_regs *regs, void *cookie);
extern void exit_signals(struct task_struct *tsk);

extern struct kmem_cache *sighand_cachep;

int unhandled_signal(struct task_struct *tsk, int sig);
void signals_init(void);






struct dentry;
struct vfsmount;

struct path {
 struct vfsmount *mnt;
 struct dentry *dentry;
};

struct fs_struct {
 atomic_t count;
 rwlock_t lock;
 int umask;
 struct path root, pwd;
};







extern struct kmem_cache *fs_cachep;

extern void exit_fs(struct task_struct *);
extern void set_fs_root(struct fs_struct *, struct path *);
extern void set_fs_pwd(struct fs_struct *, struct path *);
extern struct fs_struct *copy_fs_struct(struct fs_struct *);
extern void put_fs_struct(struct fs_struct *);







enum pid_type
{
 PIDTYPE_PID,
 PIDTYPE_PGID,
 PIDTYPE_SID,
 PIDTYPE_MAX
};
struct upid {

 int nr;
 struct pid_namespace *ns;
 struct hlist_node pid_chain;
};

struct pid
{
 atomic_t count;
 unsigned int level;

 struct hlist_head tasks[PIDTYPE_MAX];
 struct rcu_head rcu;
 struct upid numbers[1];
};

extern struct pid init_struct_pid;

struct pid_link
{
 struct hlist_node node;
 struct pid *pid;
};

static inline struct pid *get_pid(struct pid *pid)
{
 if (pid)
  ;
 return pid;
}






extern struct task_struct *pid_task(struct pid *pid, enum pid_type);
extern struct task_struct *get_pid_task(struct pid *pid, enum pid_type);

extern struct pid *get_task_pid(struct task_struct *task, enum pid_type type);





extern void attach_pid(struct task_struct *task, enum pid_type type,
   struct pid *pid);
extern void detach_pid(struct task_struct *task, enum pid_type);
extern void change_pid(struct task_struct *task, enum pid_type,
   struct pid *pid);
extern void transfer_pid(struct task_struct *old, struct task_struct *_new,
    enum pid_type);

struct pid_namespace;
extern struct pid_namespace init_pid_ns;
extern struct pid *find_pid_ns(int nr, struct pid_namespace *ns);
extern struct pid *find_vpid(int nr);




extern struct pid *find_get_pid(int nr);
extern struct pid *find_ge_pid(int nr, struct pid_namespace *);
int next_pidmap(struct pid_namespace *pid_ns, int last);

extern struct pid *alloc_pid(struct pid_namespace *ns);
extern void free_pid(struct pid *pid);
static inline struct pid_namespace *ns_of_pid(struct pid *pid)
{
 struct pid_namespace *ns = 0;
 if (pid)
  ns = pid->numbers[pid->level].ns;
 return ns;
}
static inline pid_t pid_nr(struct pid *pid)
{
 pid_t nr = 0;
 if (pid)
  nr = pid->numbers[0].nr;
 return nr;
}

pid_t pid_nr_ns(struct pid *pid, struct pid_namespace *ns);
pid_t pid_vnr(struct pid *pid);


struct percpu_counter {
 spinlock_t lock;
 s64 count;

 struct list_head list;

 s32 *counters;
};







int percpu_counter_init(struct percpu_counter *fbc, s64 amount);
int percpu_counter_init_irq(struct percpu_counter *fbc, s64 amount);
void percpu_counter_destroy(struct percpu_counter *fbc);
void percpu_counter_set(struct percpu_counter *fbc, s64 amount);
void __percpu_counter_add(struct percpu_counter *fbc, s64 amount, s32 batch);
s64 __percpu_counter_sum(struct percpu_counter *fbc, int set);

static inline void percpu_counter_add(struct percpu_counter *fbc, s64 amount)
{
 __percpu_counter_add(fbc, amount, (64*2));
}

static inline s64 percpu_counter_sum_positive(struct percpu_counter *fbc)
{
 s64 ret = __percpu_counter_sum(fbc, 0);
 return ret < 0 ? 0 : ret;
}

static inline s64 percpu_counter_sum_and_set(struct percpu_counter *fbc)
{
 return __percpu_counter_sum(fbc, 1);
}


static inline s64 percpu_counter_sum(struct percpu_counter *fbc)
{
 return __percpu_counter_sum(fbc, 0);
}

static inline s64 percpu_counter_read(struct percpu_counter *fbc)
{
 return fbc->count;
}






static inline s64 percpu_counter_read_positive(struct percpu_counter *fbc)
{
 s64 ret = fbc->count;

 __asm__ __volatile__("": : :"memory");
 if (ret >= 0)
  return ret;
 return 1;
}
static inline void percpu_counter_inc(struct percpu_counter *fbc)
{
 percpu_counter_add(fbc, 1);
}

static inline void percpu_counter_dec(struct percpu_counter *fbc)
{
 percpu_counter_add(fbc, -1);
}

static inline void percpu_counter_sub(struct percpu_counter *fbc, s64 amount)
{
 percpu_counter_add(fbc, -amount);
}



struct prop_global {





 int shift;






 struct percpu_counter events;
};






struct prop_descriptor {
 int index;
 struct prop_global pg[2];
 struct mutex mutex;
};

int prop_descriptor_init(struct prop_descriptor *pd, int shift);
void prop_change_shift(struct prop_descriptor *pd, int new_shift);





struct prop_local_percpu {



 struct percpu_counter events;




 int shift;
 unsigned long period;
 spinlock_t lock;
};

int prop_local_init_percpu(struct prop_local_percpu *pl);
void prop_local_destroy_percpu(struct prop_local_percpu *pl);
void __prop_inc_percpu(struct prop_descriptor *pd, struct prop_local_percpu *pl);
void prop_fraction_percpu(struct prop_descriptor *pd, struct prop_local_percpu *pl,
  long *numerator, long *denominator);

static inline
void prop_inc_percpu(struct prop_descriptor *pd, struct prop_local_percpu *pl)
{
 unsigned long flags;

 do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); do { (flags) = __raw_local_irq_save(); } while (0); trace_hardirqs_off(); } while (0);
 __prop_inc_percpu(pd, pl);
 do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); if (raw_irqs_disabled_flags(flags)) { raw_local_irq_restore(flags); trace_hardirqs_off(); } else { trace_hardirqs_on(); raw_local_irq_restore(flags); } } while (0);
}
void __prop_inc_percpu_max(struct prop_descriptor *pd,
      struct prop_local_percpu *pl, long frac);






struct prop_local_single {



 unsigned long events;





 int shift;
 unsigned long period;
 spinlock_t lock;
};





int prop_local_init_single(struct prop_local_single *pl);
void prop_local_destroy_single(struct prop_local_single *pl);
void __prop_inc_single(struct prop_descriptor *pd, struct prop_local_single *pl);
void prop_fraction_single(struct prop_descriptor *pd, struct prop_local_single *pl,
  long *numerator, long *denominator);

static inline
void prop_inc_single(struct prop_descriptor *pd, struct prop_local_single *pl)
{
 unsigned long flags;

 do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); do { (flags) = __raw_local_irq_save(); } while (0); trace_hardirqs_off(); } while (0);
 __prop_inc_single(pd, pl);
 do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); if (raw_irqs_disabled_flags(flags)) { raw_local_irq_restore(flags); trace_hardirqs_off(); } else { trace_hardirqs_on(); raw_local_irq_restore(flags); } } while (0);
}
























































































































































































































































































































































































































































































































































































































































































typedef struct { int mode; } seccomp_t;

extern void __secure_computing(int);
static inline void secure_computing(int this_syscall)
{
 if (__builtin_expect(!!(test_ti_thread_flag(current_thread_info(), 8)), 0))
  __secure_computing(this_syscall);
}

extern long prctl_get_seccomp(void);
extern long prctl_set_seccomp(unsigned long);

struct plist_head {
 struct list_head prio_list;
 struct list_head node_list;



};

struct plist_node {
 int prio;
 struct plist_head plist;
};
static inline void
plist_head_init(struct plist_head *head, spinlock_t *lock)
{
 INIT_LIST_HEAD(&head->prio_list);
 INIT_LIST_HEAD(&head->node_list);



}






static inline void plist_node_init(struct plist_node *node, int prio)
{
 node->prio = prio;
 plist_head_init(&node->plist, 0);
}

extern void plist_add(struct plist_node *node, struct plist_head *head);
extern void plist_del(struct plist_node *node, struct plist_head *head);
static inline int plist_head_empty( struct plist_head *head)
{
 return list_empty(&head->node_list);
}





static inline int plist_node_empty( struct plist_node *node)
{
 return plist_head_empty(&node->plist);
}
static inline struct plist_node* plist_first( struct plist_head *head)
{
 return ({ typeof( ((struct plist_node *)0)->plist.node_list ) *__mptr = (head->node_list.next); (struct plist_node *)( (char *)__mptr - ((size_t) &((struct plist_node *)0)->plist.node_list) );})
                                        ;
}
struct rt_mutex {
 spinlock_t wait_lock;
 struct plist_head wait_list;
 struct task_struct *owner;






};

struct rt_mutex_waiter;
struct hrtimer_sleeper;






 static inline int rt_mutex_debug_check_no_locks_freed( void *from,
             unsigned long len)
 {
 return 0;
 }
static inline int rt_mutex_is_locked(struct rt_mutex *lock)
{
 return lock->owner != 0;
}

extern void __rt_mutex_init(struct rt_mutex *lock, char *name);
extern void rt_mutex_destroy(struct rt_mutex *lock);

extern void rt_mutex_lock(struct rt_mutex *lock);
extern int rt_mutex_lock_interruptible(struct rt_mutex *lock,
      int detect_deadlock);
extern int rt_mutex_timed_lock(struct rt_mutex *lock,
     struct hrtimer_sleeper *timeout,
     int detect_deadlock);

extern int rt_mutex_trylock(struct rt_mutex *lock);

extern void rt_mutex_unlock(struct rt_mutex *lock);








struct task_struct;
struct rusage {
 struct timeval ru_utime;
 struct timeval ru_stime;
 long ru_maxrss;
 long ru_ixrss;
 long ru_idrss;
 long ru_isrss;
 long ru_minflt;
 long ru_majflt;
 long ru_nswap;
 long ru_inblock;
 long ru_oublock;
 long ru_msgsnd;
 long ru_msgrcv;
 long ru_nsignals;
 long ru_nvcsw;
 long ru_nivcsw;
};

struct rlimit {
 unsigned long rlim_cur;
 unsigned long rlim_max;
};

int getrusage(struct task_struct *p, int who, struct rusage *ru);


struct task_io_accounting {


 u64 rchar;

 u64 wchar;

 u64 syscr;

 u64 syscw;







 u64 read_bytes;





 u64 write_bytes;
 u64 cancelled_write_bytes;

};

struct latency_record {
 unsigned long backtrace[12];
 unsigned int count;
 unsigned long time;
 unsigned long max;
};


struct task_struct;

void account_scheduler_latency(struct task_struct *task, int usecs, int inter);

void clear_all_latency_tracing(struct task_struct *p);



struct mem_cgroup;
struct exec_domain;
struct futex_pi_state;
struct robust_list_head;
struct bio;
extern unsigned long avenrun[];
extern unsigned long total_forks;
extern int nr_threads;
extern __typeof__(unsigned long) per_cpu__process_counts;
extern int nr_processes(void);
extern unsigned long nr_running(void);
extern unsigned long nr_uninterruptible(void);
extern unsigned long nr_active(void);
extern unsigned long nr_iowait(void);

struct seq_file;
struct cfs_rq;
struct task_group;

extern void proc_sched_show_task(struct task_struct *p, struct seq_file *m);
extern void proc_sched_set_task(struct task_struct *p);
extern void
print_cfs_rq(struct seq_file *m, int cpu, struct cfs_rq *cfs_rq);
extern unsigned long long time_sync_thresh;
extern rwlock_t tasklist_lock;
extern spinlock_t mmlist_lock;

struct task_struct;

extern void sched_init(void);
extern void sched_init_smp(void);
extern void schedule_tail(struct task_struct *prev);
extern void init_idle(struct task_struct *idle, int cpu);
extern void init_idle_bootup_task(struct task_struct *idle);

extern int runqueue_is_locked(void);

extern cpumask_t nohz_cpu_mask;

extern int select_nohz_load_balancer(int cpu);







extern unsigned long rt_needs_cpu(int cpu);




extern void show_state_filter(unsigned long state_filter);

static inline void show_state(void)
{
 show_state_filter(0);
}

extern void show_regs(struct pt_regs *);






extern void show_stack(struct task_struct *task, unsigned long *sp);

void io_schedule(void);
long io_schedule_timeout(long timeout);

extern void cpu_init (void);
extern void trap_init(void);
extern void account_process_tick(struct task_struct *task, int user);
extern void update_process_times(int user);
extern void scheduler_tick(void);
extern void hrtick_resched(void);

extern void sched_show_task(struct task_struct *p);


extern void softlockup_tick(void);
extern void touch_softlockup_watchdog(void);
extern void touch_all_softlockup_watchdogs(void);
extern unsigned int softlockup_panic;
extern unsigned long sysctl_hung_task_check_count;
extern unsigned long sysctl_hung_task_timeout_secs;
extern unsigned long sysctl_hung_task_warnings;
extern int softlockup_thresh;
extern char __sched_text_start[], __sched_text_end[];


extern int in_sched_functions(unsigned long addr);


extern signed long schedule_timeout(signed long timeout);
extern signed long schedule_timeout_interruptible(signed long timeout);
extern signed long schedule_timeout_killable(signed long timeout);
extern signed long schedule_timeout_uninterruptible(signed long timeout);
 void schedule(void);

struct nsproxy;
struct user_namespace;




extern int sysctl_max_map_count;






typedef unsigned long aio_context_t;

enum {
 IOCB_CMD_PREAD = 0,
 IOCB_CMD_PWRITE = 1,
 IOCB_CMD_FSYNC = 2,
 IOCB_CMD_FDSYNC = 3,




 IOCB_CMD_NOOP = 6,
 IOCB_CMD_PREADV = 7,
 IOCB_CMD_PWRITEV = 8,
};
struct io_event {
 __u64 data;
 __u64 obj;
 __s64 res;
 __s64 res2;
};
struct iocb {

 __u64 aio_data;
 __u32 aio_key, aio_reserved1;



 __u16 aio_lio_opcode;
 __s16 aio_reqprio;
 __u32 aio_fildes;

 __u64 aio_buf;
 __u64 aio_nbytes;
 __s64 aio_offset;


 __u64 aio_reserved2;


 __u32 aio_flags;





 __u32 aio_resfd;
};
struct iovec
{
 void *iov_base;
 __kernel_size_t iov_len;
};



struct kvec {
 void *iov_base;
 size_t iov_len;
};
static inline size_t iov_length( struct iovec *iov, unsigned long nr_segs)
{
 unsigned long seg;
 size_t ret = 0;

 for (seg = 0; seg < nr_segs; seg++)
  ret += iov[seg].iov_len;
 return ret;
}

unsigned long iov_shorten(struct iovec *iov, unsigned long nr_segs, size_t to);





struct kioctx;
struct kiocb {
 struct list_head ki_run_list;
 unsigned long ki_flags;
 int ki_users;
 unsigned ki_key;

 struct file *ki_filp;
 struct kioctx *ki_ctx;
 int (*ki_cancel)(struct kiocb *, struct io_event *);
 ssize_t (*ki_retry)(struct kiocb *);
 void (*ki_dtor)(struct kiocb *);

 union {
  void *user;
  struct task_struct *tsk;
 } ki_obj;

 __u64 ki_user_data;
 wait_queue_t ki_wait;
 loff_t ki_pos;

 void *_private;

 unsigned short ki_opcode;
 size_t ki_nbytes;
 char *ki_buf;
 size_t ki_left;
 struct iovec ki_inline_vec;
  struct iovec *ki_iovec;
  unsigned long ki_nr_segs;
  unsigned long ki_cur_seg;

 struct list_head ki_list;






 struct file *ki_eventfd;
};
struct aio_ring {
 unsigned id;
 unsigned nr;
 unsigned head;
 unsigned tail;

 unsigned magic;
 unsigned compat_features;
 unsigned incompat_features;
 unsigned header_length;


 struct io_event io_events[0];
};




struct aio_ring_info {
 unsigned long mmap_base;
 unsigned long mmap_size;

 struct page **ring_pages;
 spinlock_t ring_lock;
 long nr_pages;

 unsigned nr, tail;

 struct page *internal_pages[8];
};

struct kioctx {
 atomic_t users;
 int dead;
 struct mm_struct *mm;


 unsigned long user_id;
 struct kioctx *next;

 wait_queue_head_t wait;

 spinlock_t ctx_lock;

 int reqs_active;
 struct list_head active_reqs;
 struct list_head run_list;


 unsigned max_reqs;

 struct aio_ring_info ring_info;

 struct delayed_work wq;
};


extern unsigned aio_max_size;

extern ssize_t wait_on_sync_kiocb(struct kiocb *iocb);
extern int aio_put_req(struct kiocb *iocb);
extern void kick_iocb(struct kiocb *iocb);
extern int aio_complete(struct kiocb *iocb, long res, long res2);
struct mm_struct;
extern void exit_aio(struct mm_struct *mm);





static inline struct kiocb *list_kiocb(struct list_head *h)
{
 return ({ typeof( ((struct kiocb *)0)->ki_list ) *__mptr = (h); (struct kiocb *)( (char *)__mptr - ((size_t) &((struct kiocb *)0)->ki_list) );});
}


extern unsigned long aio_nr;
extern unsigned long aio_max_nr;

extern unsigned long
arch_get_unmapped_area(struct file *, unsigned long, unsigned long,
         unsigned long, unsigned long);
extern unsigned long
arch_get_unmapped_area_topdown(struct file *filp, unsigned long addr,
     unsigned long len, unsigned long pgoff,
     unsigned long flags);
extern void arch_unmap_area(struct mm_struct *, unsigned long);
extern void arch_unmap_area_topdown(struct mm_struct *, unsigned long);
extern void set_dumpable(struct mm_struct *mm, int value);
extern int get_dumpable(struct mm_struct *mm);
struct sighand_struct {
 atomic_t count;
 struct k_sigaction action[64];
 spinlock_t siglock;
 wait_queue_head_t signalfd_wqh;
};

struct pacct_struct {
 int ac_flag;
 long ac_exitcode;
 unsigned long ac_mem;
 cputime_t ac_utime, ac_stime;
 unsigned long ac_minflt, ac_majflt;
};
struct signal_struct {
 atomic_t count;
 atomic_t live;

 wait_queue_head_t wait_chldexit;


 struct task_struct *curr_target;


 struct sigpending shared_pending;


 int group_exit_code;





 struct task_struct *group_exit_task;
 int notify_count;


 int group_stop_count;
 unsigned int flags;


 struct list_head posix_timers;


 struct hrtimer real_timer;
 struct pid *leader_pid;
 ktime_t it_real_incr;


 cputime_t it_prof_expires, it_virt_expires;
 cputime_t it_prof_incr, it_virt_incr;
 union {
  pid_t pgrp ;
  pid_t __pgrp;
 };

 struct pid *tty_old_pgrp;

 union {
  pid_t session ;
  pid_t __session;
 };


 int leader;

 struct tty_struct *tty;







 cputime_t utime, stime, cutime, cstime;
 cputime_t gtime;
 cputime_t cgtime;
 unsigned long nvcsw, nivcsw, cnvcsw, cnivcsw;
 unsigned long min_flt, maj_flt, cmin_flt, cmaj_flt;
 unsigned long inblock, oublock, cinblock, coublock;
 struct task_io_accounting ioac;







 unsigned long long sum_sched_runtime;
 struct rlimit rlim[16];

 struct list_head cpu_timers[3];




 struct key *session_keyring;
 struct key *process_keyring;


 struct pacct_struct pacct;


 struct taskstats *stats;


 unsigned audit_tty;
 struct tty_audit_buf *tty_audit_buf;

};
static inline int signal_group_exit( struct signal_struct *sig)
{
 return (sig->flags & 0x00000008) ||
  (sig->group_exit_task != 0);
}




struct user_struct {
 atomic_t __count;
 atomic_t processes;
 atomic_t files;
 atomic_t sigpending;

 atomic_t inotify_watches;
 atomic_t inotify_devs;


 atomic_t epoll_watches;



 unsigned long mq_bytes;

 unsigned long locked_shm;


 struct key *uid_keyring;
 struct key *session_keyring;



 struct hlist_node uidhash_node;
 uid_t uid;
};

extern int uids_sysfs_init(void);

extern struct user_struct *find_user(uid_t);

extern struct user_struct root_user;


struct backing_dev_info;
struct reclaim_state;


struct sched_info {

 unsigned long pcount;
 unsigned long long cpu_time,
      run_delay;


 unsigned long long last_arrival,
      last_queued;


 unsigned int bkl_count;

};



extern struct file_operations proc_schedstat_operations;



struct task_delay_info {
 spinlock_t lock;
 unsigned int flags;
 struct timespec blkio_start, blkio_end;
 u64 blkio_delay;
 u64 swapin_delay;
 u32 blkio_count;

 u32 swapin_count;


 struct timespec freepages_start, freepages_end;
 u64 freepages_delay;
 u32 freepages_count;
};


static inline int sched_info_on(void)
{

 return 1;






}

enum cpu_idle_type {
 CPU_IDLE,
 CPU_NOT_IDLE,
 CPU_NEWLY_IDLE,
 CPU_MAX_IDLE_TYPES
};
struct sched_group {
 struct sched_group *next;
 cpumask_t cpumask;






 unsigned int __cpu_power;




 u32 reciprocal_cpu_power;
};

enum sched_domain_level {
 SD_LV_NONE = 0,
 SD_LV_SIBLING,
 SD_LV_MC,
 SD_LV_CPU,
 SD_LV_NODE,
 SD_LV_ALLNODES,
 SD_LV_MAX
};

struct sched_domain_attr {
 int relax_domain_level;
};





struct sched_domain {

 struct sched_domain *parent;
 struct sched_domain *child;
 struct sched_group *groups;
 cpumask_t span;
 unsigned long min_interval;
 unsigned long max_interval;
 unsigned int busy_factor;
 unsigned int imbalance_pct;
 unsigned int cache_nice_tries;
 unsigned int busy_idx;
 unsigned int idle_idx;
 unsigned int newidle_idx;
 unsigned int wake_idx;
 unsigned int forkexec_idx;
 int flags;
 enum sched_domain_level level;


 unsigned long last_balance;
 unsigned int balance_interval;
 unsigned int nr_balance_failed;

 u64 last_update;



 unsigned int lb_count[CPU_MAX_IDLE_TYPES];
 unsigned int lb_failed[CPU_MAX_IDLE_TYPES];
 unsigned int lb_balanced[CPU_MAX_IDLE_TYPES];
 unsigned int lb_imbalance[CPU_MAX_IDLE_TYPES];
 unsigned int lb_gained[CPU_MAX_IDLE_TYPES];
 unsigned int lb_hot_gained[CPU_MAX_IDLE_TYPES];
 unsigned int lb_nobusyg[CPU_MAX_IDLE_TYPES];
 unsigned int lb_nobusyq[CPU_MAX_IDLE_TYPES];


 unsigned int alb_count;
 unsigned int alb_failed;
 unsigned int alb_pushed;


 unsigned int sbe_count;
 unsigned int sbe_balanced;
 unsigned int sbe_pushed;


 unsigned int sbf_count;
 unsigned int sbf_balanced;
 unsigned int sbf_pushed;


 unsigned int ttwu_wake_remote;
 unsigned int ttwu_move_affine;
 unsigned int ttwu_move_balance;

};

extern void partition_sched_domains(int ndoms_new, cpumask_t *doms_new,
        struct sched_domain_attr *dattr_new);
extern int arch_reinit_sched_domains(void);
struct io_context;


struct group_info {
 int ngroups;
 atomic_t usage;
 gid_t small_block[32];
 int nblocks;
 gid_t *blocks[0];
};
extern struct group_info *groups_alloc(int gidsetsize);





extern int set_current_groups(struct group_info *group_info);
extern int groups_search(struct group_info *group_info, gid_t grp);







static inline void prefetch_stack(struct task_struct *t) { }


struct audit_context;
struct mempolicy;
struct pipe_inode_info;
struct uts_namespace;

struct rq;
struct sched_domain;

struct sched_class {
 struct sched_class *next;

 void (*enqueue_task) (struct rq *rq, struct task_struct *p, int wakeup);
 void (*dequeue_task) (struct rq *rq, struct task_struct *p, int sleep);
 void (*yield_task) (struct rq *rq);
 int (*select_task_rq)(struct task_struct *p, int sync);

 void (*check_preempt_curr) (struct rq *rq, struct task_struct *p);

 struct task_struct * (*pick_next_task) (struct rq *rq);
 void (*put_prev_task) (struct rq *rq, struct task_struct *p);


 unsigned long (*load_balance) (struct rq *this_rq, int this_cpu,
   struct rq *busiest, unsigned long max_load_move,
   struct sched_domain *sd, enum cpu_idle_type idle,
   int *all_pinned, int *this_best_prio);

 int (*move_one_task) (struct rq *this_rq, int this_cpu,
         struct rq *busiest, struct sched_domain *sd,
         enum cpu_idle_type idle);
 void (*pre_schedule) (struct rq *this_rq, struct task_struct *task);
 void (*post_schedule) (struct rq *this_rq);
 void (*task_wake_up) (struct rq *this_rq, struct task_struct *task);


 void (*set_curr_task) (struct rq *rq);
 void (*task_tick) (struct rq *rq, struct task_struct *p, int queued);
 void (*task_new) (struct rq *rq, struct task_struct *p);
 void (*set_cpus_allowed)(struct task_struct *p,
     cpumask_t *newmask);

 void (*rq_online)(struct rq *rq);
 void (*rq_offline)(struct rq *rq);

 void (*switched_from) (struct rq *this_rq, struct task_struct *task,
          int running);
 void (*switched_to) (struct rq *this_rq, struct task_struct *task,
        int running);
 void (*prio_changed) (struct rq *this_rq, struct task_struct *task,
        int oldprio, int running);


 void (*moved_group) (struct task_struct *p);

};

struct load_weight {
 unsigned long weight, inv_weight;
};
struct sched_entity {
 struct load_weight load;
 struct rb_node run_node;
 struct list_head group_node;
 unsigned int on_rq;

 u64 exec_start;
 u64 sum_exec_runtime;
 u64 vruntime;
 u64 prev_sum_exec_runtime;

 u64 last_wakeup;
 u64 avg_overlap;


 u64 wait_start;
 u64 wait_max;
 u64 wait_count;
 u64 wait_sum;

 u64 sleep_start;
 u64 sleep_max;
 s64 sum_sleep_runtime;

 u64 block_start;
 u64 block_max;
 u64 exec_max;
 u64 slice_max;

 u64 nr_migrations;
 u64 nr_migrations_cold;
 u64 nr_failed_migrations_affine;
 u64 nr_failed_migrations_running;
 u64 nr_failed_migrations_hot;
 u64 nr_forced_migrations;
 u64 nr_forced2_migrations;

 u64 nr_wakeups;
 u64 nr_wakeups_sync;
 u64 nr_wakeups_migrate;
 u64 nr_wakeups_local;
 u64 nr_wakeups_remote;
 u64 nr_wakeups_affine;
 u64 nr_wakeups_affine_attempts;
 u64 nr_wakeups_passive;
 u64 nr_wakeups_idle;



 struct sched_entity *parent;

 struct cfs_rq *cfs_rq;

 struct cfs_rq *my_q;

};

struct sched_rt_entity {
 struct list_head run_list;
 unsigned int time_slice;
 unsigned long timeout;
 int nr_cpus_allowed;

 struct sched_rt_entity *back;

 struct sched_rt_entity *parent;

 struct rt_rq *rt_rq;

 struct rt_rq *my_q;

};

struct task_struct {
 long state;
 void *stack;
 atomic_t usage;
 unsigned int flags;
 unsigned int ptrace;

 int lock_depth;







 int prio, static_prio, normal_prio;
 unsigned int rt_priority;
 struct sched_class *sched_class;
 struct sched_entity se;
 struct sched_rt_entity rt;



 struct hlist_head preempt_notifiers;
 unsigned char fpu_counter;
 s8 oomkilladj;

 unsigned int btrace_seq;


 unsigned int policy;
 cpumask_t cpus_allowed;







 struct sched_info sched_info;


 struct list_head tasks;

 struct mm_struct *mm, *active_mm;


 struct linux_binfmt *binfmt;
 int exit_state;
 int exit_code, exit_signal;
 int pdeath_signal;

 unsigned int personality;
 unsigned did_exec:1;
 pid_t pid;
 pid_t tgid;
 struct task_struct *real_parent;
 struct task_struct *parent;



 struct list_head children;
 struct list_head sibling;
 struct task_struct *group_leader;






 struct list_head ptraced;
 struct list_head ptrace_entry;


 struct pid_link pids[PIDTYPE_MAX];
 struct list_head thread_group;

 struct completion *vfork_done;
 int *set_child_tid;
 int *clear_child_tid;

 cputime_t utime, stime, utimescaled, stimescaled;
 cputime_t gtime;
 cputime_t prev_utime, prev_stime;
 unsigned long nvcsw, nivcsw;
 struct timespec start_time;
 struct timespec real_start_time;

 unsigned long min_flt, maj_flt;

   cputime_t it_prof_expires, it_virt_expires;
 unsigned long long it_sched_expires;
 struct list_head cpu_timers[3];


 uid_t uid,euid,suid,fsuid;
 gid_t gid,egid,sgid,fsgid;
 struct group_info *group_info;
 kernel_cap_t cap_effective, cap_inheritable, cap_permitted, cap_bset;
 struct user_struct *user;
 unsigned securebits;

 unsigned char jit_keyring;
 struct key *request_key_auth;
 struct key *thread_keyring;

 char comm[16];




 int link_count, total_link_count;


 struct sysv_sem sysvsem;



 unsigned long last_switch_timestamp;
 unsigned long last_switch_count;


 struct thread_struct thread;

 struct fs_struct *fs;

 struct files_struct *files;

 struct nsproxy *nsproxy;

 struct signal_struct *signal;
 struct sighand_struct *sighand;

 sigset_t blocked, real_blocked;
 sigset_t saved_sigmask;
 struct sigpending pending;

 unsigned long sas_ss_sp;
 size_t sas_ss_size;
 int (*notifier)(void *priv);
 void *notifier_data;
 sigset_t *notifier_mask;

 void *security;

 struct audit_context *audit_context;

 uid_t loginuid;
 unsigned int sessionid;

 seccomp_t seccomp;


    u32 parent_exec_id;
    u32 self_exec_id;

 spinlock_t alloc_lock;


 spinlock_t pi_lock;



 struct plist_head pi_waiters;

 struct rt_mutex_waiter *pi_blocked_on;







 unsigned int irq_events;
 int hardirqs_enabled;
 unsigned long hardirq_enable_ip;
 unsigned int hardirq_enable_event;
 unsigned long hardirq_disable_ip;
 unsigned int hardirq_disable_event;
 int softirqs_enabled;
 unsigned long softirq_disable_ip;
 unsigned int softirq_disable_event;
 unsigned long softirq_enable_ip;
 unsigned int softirq_enable_event;
 int hardirq_context;
 int softirq_context;
 void *journal_info;


 struct bio *bio_list, **bio_tail;


 struct reclaim_state *reclaim_state;

 struct backing_dev_info *backing_dev_info;

 struct io_context *io_context;

 unsigned long ptrace_message;
 siginfo_t *last_siginfo;
 struct task_io_accounting ioac;

 u64 acct_rss_mem1;
 u64 acct_vm_mem1;
 cputime_t acct_timexpd;


 nodemask_t mems_allowed;
 int cpuset_mems_generation;
 int cpuset_mem_spread_rotor;



 struct css_set *cgroups;

 struct list_head cg_list;


 struct robust_list_head *robust_list;

 struct compat_robust_list_head *compat_robust_list;

 struct list_head pi_state_list;
 struct futex_pi_state *pi_state_cache;


 struct mempolicy *mempolicy;
 short il_next;

 atomic_t fs_excl;
 struct rcu_head rcu;

 struct list_head *scm_work_list;




 struct pipe_inode_info *splice_pipe;

 struct task_delay_info *delays;




 struct prop_local_single dirties;

 int latency_record_count;
 struct latency_record latency_record[32];

};
static inline int rt_prio(int prio)
{
 if (__builtin_expect(!!(prio < 100), 0))
  return 1;
 return 0;
}

static inline int rt_task(struct task_struct *p)
{
 return rt_prio(p->prio);
}

static inline void set_task_session(struct task_struct *tsk, pid_t session)
{
 tsk->signal->__session = session;
}

static inline void set_task_pgrp(struct task_struct *tsk, pid_t pgrp)
{
 tsk->signal->__pgrp = pgrp;
}

static inline struct pid *task_pid(struct task_struct *task)
{
 return task->pids[PIDTYPE_PID].pid;
}

static inline struct pid *task_tgid(struct task_struct *task)
{
 return task->group_leader->pids[PIDTYPE_PID].pid;
}

static inline struct pid *task_pgrp(struct task_struct *task)
{
 return task->group_leader->pids[PIDTYPE_PGID].pid;
}

static inline struct pid *task_session(struct task_struct *task)
{
 return task->group_leader->pids[PIDTYPE_SID].pid;
}

struct pid_namespace;
static inline pid_t task_pid_nr(struct task_struct *tsk)
{
 return tsk->pid;
}

pid_t task_pid_nr_ns(struct task_struct *tsk, struct pid_namespace *ns);

static inline pid_t task_pid_vnr(struct task_struct *tsk)
{
 return pid_vnr(task_pid(tsk));
}


static inline pid_t task_tgid_nr(struct task_struct *tsk)
{
 return tsk->tgid;
}

pid_t task_tgid_nr_ns(struct task_struct *tsk, struct pid_namespace *ns);

static inline pid_t task_tgid_vnr(struct task_struct *tsk)
{
 return pid_vnr(task_tgid(tsk));
}


static inline pid_t task_pgrp_nr(struct task_struct *tsk)
{
 return tsk->signal->__pgrp;
}

pid_t task_pgrp_nr_ns(struct task_struct *tsk, struct pid_namespace *ns);

static inline pid_t task_pgrp_vnr(struct task_struct *tsk)
{
 return pid_vnr(task_pgrp(tsk));
}


static inline pid_t task_session_nr(struct task_struct *tsk)
{
 return tsk->signal->__session;
}

pid_t task_session_nr_ns(struct task_struct *tsk, struct pid_namespace *ns);

static inline pid_t task_session_vnr(struct task_struct *tsk)
{
 return pid_vnr(task_session(tsk));
}
static inline int pid_alive(struct task_struct *p)
{
 return p->pids[PIDTYPE_PID].pid != 0;
}







static inline int is_global_init(struct task_struct *tsk)
{
 return tsk->pid == 1;
}





extern int is_container_init(struct task_struct *tsk);

extern struct pid *cad_pid;

extern void free_task(struct task_struct *tsk);


extern void __put_task_struct(struct task_struct *t);

static inline void put_task_struct(struct task_struct *t)
{




}

extern cputime_t task_utime(struct task_struct *p);
extern cputime_t task_stime(struct task_struct *p);
extern cputime_t task_gtime(struct task_struct *p);
extern int set_cpus_allowed_ptr(struct task_struct *p,
    cpumask_t *new_mask);
static inline int set_cpus_allowed(struct task_struct *p, cpumask_t new_mask)
{
 return set_cpus_allowed_ptr(p, &new_mask);
}

extern unsigned long long sched_clock(void);

extern void sched_clock_init(void);
extern u64 sched_clock_cpu(int cpu);
extern void sched_clock_tick(void);
extern void sched_clock_idle_sleep_event(void);
extern void sched_clock_idle_wakeup_event(u64 delta_ns);






extern unsigned long long cpu_clock(int cpu);

extern unsigned long long
task_sched_runtime(struct task_struct *task);



extern void sched_exec(void);




extern void sched_clock_idle_sleep_event(void);
extern void sched_clock_idle_wakeup_event(u64 delta_ns);


extern void idle_task_exit(void);




extern void sched_idle_next(void);


extern void wake_up_idle_cpu(int cpu);





extern unsigned int sysctl_sched_latency;
extern unsigned int sysctl_sched_min_granularity;
extern unsigned int sysctl_sched_wakeup_granularity;
extern unsigned int sysctl_sched_child_runs_first;
extern unsigned int sysctl_sched_features;
extern unsigned int sysctl_sched_migration_cost;
extern unsigned int sysctl_sched_nr_migrate;
extern unsigned int sysctl_sched_shares_ratelimit;

int sched_nr_latency_handler(struct ctl_table *table, int write,
  struct file *file, void *buffer, size_t *length,
  loff_t *ppos);

extern unsigned int sysctl_sched_rt_period;
extern int sysctl_sched_rt_runtime;

int sched_rt_handler(struct ctl_table *table, int write,
  struct file *filp, void *buffer, size_t *lenp,
  loff_t *ppos);

extern unsigned int sysctl_sched_compat_yield;


extern int rt_mutex_getprio(struct task_struct *p);
extern void rt_mutex_setprio(struct task_struct *p, int prio);
extern void rt_mutex_adjust_pi(struct task_struct *p);
extern void set_user_nice(struct task_struct *p, long nice);
extern int task_prio( struct task_struct *p);
extern int task_nice( struct task_struct *p);
extern int can_nice( struct task_struct *p, int nice);
extern int task_curr( struct task_struct *p);
extern int idle_cpu(int cpu);
extern int sched_setscheduler(struct task_struct *, int, struct sched_param *);
extern int sched_setscheduler_nocheck(struct task_struct *, int,
          struct sched_param *);
extern struct task_struct *idle_task(int cpu);
extern struct task_struct *curr_task(int cpu);
extern void set_curr_task(int cpu, struct task_struct *p);

void yield(void);




extern struct exec_domain default_exec_domain;

union thread_union {
 struct thread_info thread_info;
 unsigned long stack[(((1UL) << 12) << 1)/sizeof(long)];
};


static inline int kstack_end(void *addr)
{



 return !(((unsigned long)addr+sizeof(void*)-1) & ((((1UL) << 12) << 1)-sizeof(void*)));
}


extern union thread_union init_thread_union;
extern struct task_struct init_task;

extern struct mm_struct init_mm;

extern struct pid_namespace init_pid_ns;
extern struct task_struct *find_task_by_pid_type_ns(int type, int pid,
  struct pid_namespace *ns);

extern struct task_struct *find_task_by_vpid(pid_t nr);
extern struct task_struct *find_task_by_pid_ns(pid_t nr,
  struct pid_namespace *ns);

extern void __set_special_pids(struct pid *pid);


extern struct user_struct * alloc_uid(struct user_namespace *, uid_t);
static inline struct user_struct *get_uid(struct user_struct *u)
{
 ;
 return u;
}
extern void free_uid(struct user_struct *);
extern void switch_uid(struct user_struct *);
extern void release_uids(struct user_namespace *ns);



extern void do_timer(unsigned long ticks);

extern int wake_up_state(struct task_struct *tsk, unsigned int state);
extern int wake_up_process(struct task_struct *tsk);
extern void wake_up_new_task(struct task_struct *tsk,
    unsigned long clone_flags);

 extern void kick_process(struct task_struct *tsk);



extern void sched_fork(struct task_struct *p, int clone_flags);
extern void sched_dead(struct task_struct *p);

extern int in_group_p(gid_t);
extern int in_egroup_p(gid_t);

extern void proc_caches_init(void);
extern void flush_signals(struct task_struct *);
extern void ignore_signals(struct task_struct *);
extern void flush_signal_handlers(struct task_struct *, int force_default);
extern int dequeue_signal(struct task_struct *tsk, sigset_t *mask, siginfo_t *info);

static inline int dequeue_signal_lock(struct task_struct *tsk, sigset_t *mask, siginfo_t *info)
{
 unsigned long flags;
 int ret;

 ;
 ret = dequeue_signal(tsk, mask, info);
 ;

 return ret;
}

extern void block_all_signals(int (*notifier)(void *priv), void *priv,
         sigset_t *mask);
extern void unblock_all_signals(void);
extern void release_task(struct task_struct * p);
extern int send_sig_info(int, struct siginfo *, struct task_struct *);
extern int force_sigsegv(int, struct task_struct *);
extern int force_sig_info(int, struct siginfo *, struct task_struct *);
extern int __kill_pgrp_info(int sig, struct siginfo *info, struct pid *pgrp);
extern int kill_pid_info(int sig, struct siginfo *info, struct pid *pid);
extern int kill_pid_info_as_uid(int, struct siginfo *, struct pid *, uid_t, uid_t, u32);
extern int kill_pgrp(struct pid *pid, int sig, int priv);
extern int kill_pid(struct pid *pid, int sig, int priv);
extern int kill_proc_info(int, struct siginfo *, pid_t);
extern int do_notify_parent(struct task_struct *, int);
extern void force_sig(int, struct task_struct *);
extern void force_sig_specific(int, struct task_struct *);
extern int send_sig(int, struct task_struct *, int);
extern void zap_other_threads(struct task_struct *p);
extern struct sigqueue *sigqueue_alloc(void);
extern void sigqueue_free(struct sigqueue *);
extern int send_sigqueue(struct sigqueue *, struct task_struct *, int group);
extern int do_sigaction(int, struct k_sigaction *, struct k_sigaction *);
extern int do_sigaltstack( stack_t *, stack_t *, unsigned long);

static inline int kill_cad_pid(int sig, int priv)
{
 return kill_pid(cad_pid, sig, priv);
}






static inline int is_si_special( struct siginfo *info)
{
 return info <= ((struct siginfo *) 2);
}



static inline int on_sig_stack(unsigned long sp)
{
 return (sp - get_current()->sas_ss_sp < get_current()->sas_ss_size);
}

static inline int sas_ss_flags(unsigned long sp)
{
 return (get_current()->sas_ss_size == 0 ? 2
  : on_sig_stack(sp) ? 1 : 0);
}




extern struct mm_struct * mm_alloc(void);


extern void __mmdrop(struct mm_struct *);
static inline void mmdrop(struct mm_struct * mm)
{
 if (__builtin_expect(!!(1), 0))
  __mmdrop(mm);
}


extern void mmput(struct mm_struct *);

extern struct mm_struct *get_task_mm(struct task_struct *task);

extern void mm_release(struct task_struct *, struct mm_struct *);

extern struct mm_struct *dup_mm(struct task_struct *tsk);

extern int copy_thread(int, unsigned long, unsigned long, unsigned long, struct task_struct *, struct pt_regs *);
extern void flush_thread(void);
extern void exit_thread(void);

extern void exit_files(struct task_struct *);
extern void __cleanup_signal(struct signal_struct *);
extern void __cleanup_sighand(struct sighand_struct *);

extern void exit_itimers(struct signal_struct *);
extern void flush_itimer_signals(void);

extern void do_group_exit(int);

extern void daemonize( char *, ...);
extern int allow_signal(int);
extern int disallow_signal(int);

extern int do_execve(char *, char * *, char * *, struct pt_regs *);
extern long do_fork(unsigned long, unsigned long, struct pt_regs *, unsigned long, int *, int *);
struct task_struct *fork_idle(int);

extern void set_task_comm(struct task_struct *tsk, char *from);
extern char *get_task_comm(char *to, struct task_struct *tsk);


extern unsigned long wait_task_inactive(struct task_struct *, long match_state);
static inline int has_group_leader_pid(struct task_struct *p)
{
 return p->pid == p->tgid;
}

static inline
int same_thread_group(struct task_struct *p1, struct task_struct *p2)
{
 return p1->tgid == p2->tgid;
}

static inline struct task_struct *next_thread( struct task_struct *p)
{
 return ({ typeof( ((struct task_struct *)0)->thread_group ) *__mptr = ((p->thread_group.next)); (struct task_struct *)( (char *)__mptr - ((size_t) &((struct task_struct *)0)->thread_group) );})
                                      ;
}

static inline int thread_group_empty(struct task_struct *p)
{
 return list_empty(&p->thread_group);
}
static inline void setup_thread_stack(struct task_struct *p, struct task_struct *org)
{
 *((struct thread_info *)(p)->stack) = *((struct thread_info *)(org)->stack);
 ((struct thread_info *)(p)->stack)->task = p;
}

static inline unsigned long *end_of_stack(struct task_struct *p)
{
 return (unsigned long *)(((struct thread_info *)(p)->stack) + 1);
}



static inline int object_is_on_stack(void *obj)
{
 void *stack = ((get_current())->stack);

 return (obj >= stack) && (obj < (stack + (((1UL) << 12) << 1)));
}

extern void thread_info_cache_init(void);




static inline void set_tsk_thread_flag(struct task_struct *tsk, int flag)
{
 set_ti_thread_flag(((struct thread_info *)(tsk)->stack), flag);
}

static inline void clear_tsk_thread_flag(struct task_struct *tsk, int flag)
{
 clear_ti_thread_flag(((struct thread_info *)(tsk)->stack), flag);
}

static inline int test_and_set_tsk_thread_flag(struct task_struct *tsk, int flag)
{
 return test_and_set_ti_thread_flag(((struct thread_info *)(tsk)->stack), flag);
}

static inline int test_and_clear_tsk_thread_flag(struct task_struct *tsk, int flag)
{
 return test_and_clear_ti_thread_flag(((struct thread_info *)(tsk)->stack), flag);
}

static inline int test_tsk_thread_flag(struct task_struct *tsk, int flag)
{
 return test_ti_thread_flag(((struct thread_info *)(tsk)->stack), flag);
}

static inline void set_tsk_need_resched(struct task_struct *tsk)
{
 set_tsk_thread_flag(tsk,3);
}

static inline void clear_tsk_need_resched(struct task_struct *tsk)
{
 clear_tsk_thread_flag(tsk,3);
}

static inline int test_tsk_need_resched(struct task_struct *tsk)
{
 return __builtin_expect(!!(test_tsk_thread_flag(tsk,3)), 0);
}

static inline int signal_pending(struct task_struct *p)
{
 return __builtin_expect(!!(test_tsk_thread_flag(p,2)), 0);
}

extern int __fatal_signal_pending(struct task_struct *p);

static inline int fatal_signal_pending(struct task_struct *p)
{
 return signal_pending(p) && __fatal_signal_pending(p);
}

static inline int signal_pending_state(long state, struct task_struct *p)
{
 if (!(state & (1 | 128)))
  return 0;
 if (!signal_pending(p))
  return 0;

 return (state & 1) || __fatal_signal_pending(p);
}

static inline int need_resched(void)
{
 return __builtin_expect(!!(test_ti_thread_flag(current_thread_info(), 3)), 0);
}
extern int _cond_resched(void);






static inline int cond_resched(void)
{



 return 0;

}

extern int cond_resched_lock(spinlock_t * lock);
extern int cond_resched_softirq(void);
static inline int cond_resched_bkl(void)
{
 return _cond_resched();
}






static inline int spin_needbreak(spinlock_t *lock)
{



 return 0;

}







extern void recalc_sigpending_and_wake(struct task_struct *t);
extern void recalc_sigpending(void);

extern void signal_wake_up(struct task_struct *t, int resume_stopped);






static inline unsigned int task_cpu( struct task_struct *p)
{
 return ((struct thread_info *)(p)->stack)->cpu;
}

extern void set_task_cpu(struct task_struct *p, unsigned int cpu);
extern void arch_pick_mmap_layout(struct mm_struct *mm);


extern void
__trace_special(void *__tr, void *__data,
  unsigned long arg1, unsigned long arg2, unsigned long arg3);
extern long sched_setaffinity(pid_t pid, cpumask_t *new_mask);
extern long sched_getaffinity(pid_t pid, cpumask_t *mask);

extern int sched_mc_power_savings, sched_smt_power_savings;

extern void normalize_rt_tasks(void);



extern struct task_group init_task_group;




extern struct task_group *sched_create_group(struct task_group *parent);
extern void sched_destroy_group(struct task_group *tg);
extern void sched_move_task(struct task_struct *tsk);

extern int sched_group_set_shares(struct task_group *tg, unsigned long shares);
extern unsigned long sched_group_shares(struct task_group *tg);


extern int sched_group_set_rt_runtime(struct task_group *tg,
          long rt_runtime_us);
extern long sched_group_rt_runtime(struct task_group *tg);
extern int sched_group_set_rt_period(struct task_group *tg,
          long rt_period_us);
extern long sched_group_rt_period(struct task_group *tg);




static inline void add_rchar(struct task_struct *tsk, ssize_t amt)
{
 tsk->ioac.rchar += amt;
}

static inline void add_wchar(struct task_struct *tsk, ssize_t amt)
{
 tsk->ioac.wchar += amt;
}

static inline void inc_syscr(struct task_struct *tsk)
{
 tsk->ioac.syscr++;
}

static inline void inc_syscw(struct task_struct *tsk)
{
 tsk->ioac.syscw++;
}
static inline void mm_update_next_owner(struct mm_struct *mm)
{
}

static inline void mm_init_owner(struct mm_struct *mm, struct task_struct *p)
{
}


struct file;
struct completion;






struct __sysctl_args {
 int *name;
 int nlen;
 void *oldval;
 size_t *oldlenp;
 void *newval;
 size_t newlen;
 unsigned long __unused[4];
};
enum
{
 CTL_KERN=1,
 CTL_VM=2,
 CTL_NET=3,
 CTL_PROC=4,
 CTL_FS=5,
 CTL_DEBUG=6,
 CTL_DEV=7,
 CTL_BUS=8,
 CTL_ABI=9,
 CTL_CPU=10,
 CTL_ARLAN=254,
 CTL_S390DBF=5677,
 CTL_SUNRPC=7249,
 CTL_PM=9899,
 CTL_FRV=9898,
};


enum
{
 CTL_BUS_ISA=1
};


enum
{
 INOTIFY_MAX_USER_INSTANCES=1,
 INOTIFY_MAX_USER_WATCHES=2,
 INOTIFY_MAX_QUEUED_EVENTS=3
};


enum
{
 KERN_OSTYPE=1,
 KERN_OSRELEASE=2,
 KERN_OSREV=3,
 KERN_VERSION=4,
 KERN_SECUREMASK=5,
 KERN_PROF=6,
 KERN_NODENAME=7,
 KERN_DOMAINNAME=8,

 KERN_PANIC=15,
 KERN_REALROOTDEV=16,

 KERN_SPARC_REBOOT=21,
 KERN_CTLALTDEL=22,
 KERN_PRINTK=23,
 KERN_NAMETRANS=24,
 KERN_PPC_HTABRECLAIM=25,
 KERN_PPC_ZEROPAGED=26,
 KERN_PPC_POWERSAVE_NAP=27,
 KERN_MODPROBE=28,
 KERN_SG_BIG_BUFF=29,
 KERN_ACCT=30,
 KERN_PPC_L2CR=31,

 KERN_RTSIGNR=32,
 KERN_RTSIGMAX=33,

 KERN_SHMMAX=34,
 KERN_MSGMAX=35,
 KERN_MSGMNB=36,
 KERN_MSGPOOL=37,
 KERN_SYSRQ=38,
 KERN_MAX_THREADS=39,
  KERN_RANDOM=40,
  KERN_SHMALL=41,
  KERN_MSGMNI=42,
  KERN_SEM=43,
  KERN_SPARC_STOP_A=44,
  KERN_SHMMNI=45,
 KERN_OVERFLOWUID=46,
 KERN_OVERFLOWGID=47,
 KERN_SHMPATH=48,
 KERN_HOTPLUG=49,
 KERN_IEEE_EMULATION_WARNINGS=50,
 KERN_S390_USER_DEBUG_LOGGING=51,
 KERN_CORE_USES_PID=52,
 KERN_TAINTED=53,
 KERN_CADPID=54,
 KERN_PIDMAX=55,
   KERN_CORE_PATTERN=56,
 KERN_PANIC_ON_OOPS=57,
 KERN_HPPA_PWRSW=58,
 KERN_HPPA_UNALIGNED=59,
 KERN_PRINTK_RATELIMIT=60,
 KERN_PRINTK_RATELIMIT_BURST=61,
 KERN_PTY=62,
 KERN_NGROUPS_MAX=63,
 KERN_SPARC_SCONS_PWROFF=64,
 KERN_HZ_TIMER=65,
 KERN_UNKNOWN_NMI_PANIC=66,
 KERN_BOOTLOADER_TYPE=67,
 KERN_RANDOMIZE=68,
 KERN_SETUID_DUMPABLE=69,
 KERN_SPIN_RETRY=70,
 KERN_ACPI_VIDEO_FLAGS=71,
 KERN_IA64_UNALIGNED=72,
 KERN_COMPAT_LOG=73,
 KERN_MAX_LOCK_DEPTH=74,
 KERN_NMI_WATCHDOG=75,
 KERN_PANIC_ON_NMI=76,
};




enum
{
 VM_UNUSED1=1,
 VM_UNUSED2=2,
 VM_UNUSED3=3,
 VM_UNUSED4=4,
 VM_OVERCOMMIT_MEMORY=5,
 VM_UNUSED5=6,
 VM_UNUSED7=7,
 VM_UNUSED8=8,
 VM_UNUSED9=9,
 VM_PAGE_CLUSTER=10,
 VM_DIRTY_BACKGROUND=11,
 VM_DIRTY_RATIO=12,
 VM_DIRTY_WB_CS=13,
 VM_DIRTY_EXPIRE_CS=14,
 VM_NR_PDFLUSH_THREADS=15,
 VM_OVERCOMMIT_RATIO=16,
 VM_PAGEBUF=17,
 VM_HUGETLB_PAGES=18,
 VM_SWAPPINESS=19,
 VM_LOWMEM_RESERVE_RATIO=20,
 VM_MIN_FREE_KBYTES=21,
 VM_MAX_MAP_COUNT=22,
 VM_LAPTOP_MODE=23,
 VM_BLOCK_DUMP=24,
 VM_HUGETLB_GROUP=25,
 VM_VFS_CACHE_PRESSURE=26,
 VM_LEGACY_VA_LAYOUT=27,
 VM_SWAP_TOKEN_TIMEOUT=28,
 VM_DROP_PAGECACHE=29,
 VM_PERCPU_PAGELIST_FRACTION=30,
 VM_ZONE_RECLAIM_MODE=31,
 VM_MIN_UNMAPPED=32,
 VM_PANIC_ON_OOM=33,
 VM_VDSO_ENABLED=34,
 VM_MIN_SLAB=35,
};



enum
{
 NET_CORE=1,
 NET_ETHER=2,
 NET_802=3,
 NET_UNIX=4,
 NET_IPV4=5,
 NET_IPX=6,
 NET_ATALK=7,
 NET_NETROM=8,
 NET_AX25=9,
 NET_BRIDGE=10,
 NET_ROSE=11,
 NET_IPV6=12,
 NET_X25=13,
 NET_TR=14,
 NET_DECNET=15,
 NET_ECONET=16,
 NET_SCTP=17,
 NET_LLC=18,
 NET_NETFILTER=19,
 NET_DCCP=20,
 NET_IRDA=412,
};


enum
{
 RANDOM_POOLSIZE=1,
 RANDOM_ENTROPY_COUNT=2,
 RANDOM_READ_THRESH=3,
 RANDOM_WRITE_THRESH=4,
 RANDOM_BOOT_ID=5,
 RANDOM_UUID=6
};


enum
{
 PTY_MAX=1,
 PTY_NR=2
};


enum
{
 BUS_ISA_MEM_BASE=1,
 BUS_ISA_PORT_BASE=2,
 BUS_ISA_PORT_SHIFT=3
};


enum
{
 NET_CORE_WMEM_MAX=1,
 NET_CORE_RMEM_MAX=2,
 NET_CORE_WMEM_DEFAULT=3,
 NET_CORE_RMEM_DEFAULT=4,

 NET_CORE_MAX_BACKLOG=6,
 NET_CORE_FASTROUTE=7,
 NET_CORE_MSG_COST=8,
 NET_CORE_MSG_BURST=9,
 NET_CORE_OPTMEM_MAX=10,
 NET_CORE_HOT_LIST_LENGTH=11,
 NET_CORE_DIVERT_VERSION=12,
 NET_CORE_NO_CONG_THRESH=13,
 NET_CORE_NO_CONG=14,
 NET_CORE_LO_CONG=15,
 NET_CORE_MOD_CONG=16,
 NET_CORE_DEV_WEIGHT=17,
 NET_CORE_SOMAXCONN=18,
 NET_CORE_BUDGET=19,
 NET_CORE_AEVENT_ETIME=20,
 NET_CORE_AEVENT_RSEQTH=21,
 NET_CORE_WARNINGS=22,
};







enum
{
 NET_UNIX_DESTROY_DELAY=1,
 NET_UNIX_DELETE_DELAY=2,
 NET_UNIX_MAX_DGRAM_QLEN=3,
};


enum
{
 NET_NF_CONNTRACK_MAX=1,
 NET_NF_CONNTRACK_TCP_TIMEOUT_SYN_SENT=2,
 NET_NF_CONNTRACK_TCP_TIMEOUT_SYN_RECV=3,
 NET_NF_CONNTRACK_TCP_TIMEOUT_ESTABLISHED=4,
 NET_NF_CONNTRACK_TCP_TIMEOUT_FIN_WAIT=5,
 NET_NF_CONNTRACK_TCP_TIMEOUT_CLOSE_WAIT=6,
 NET_NF_CONNTRACK_TCP_TIMEOUT_LAST_ACK=7,
 NET_NF_CONNTRACK_TCP_TIMEOUT_TIME_WAIT=8,
 NET_NF_CONNTRACK_TCP_TIMEOUT_CLOSE=9,
 NET_NF_CONNTRACK_UDP_TIMEOUT=10,
 NET_NF_CONNTRACK_UDP_TIMEOUT_STREAM=11,
 NET_NF_CONNTRACK_ICMP_TIMEOUT=12,
 NET_NF_CONNTRACK_GENERIC_TIMEOUT=13,
 NET_NF_CONNTRACK_BUCKETS=14,
 NET_NF_CONNTRACK_LOG_INVALID=15,
 NET_NF_CONNTRACK_TCP_TIMEOUT_MAX_RETRANS=16,
 NET_NF_CONNTRACK_TCP_LOOSE=17,
 NET_NF_CONNTRACK_TCP_BE_LIBERAL=18,
 NET_NF_CONNTRACK_TCP_MAX_RETRANS=19,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_CLOSED=20,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_WAIT=21,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_ECHOED=22,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_ESTABLISHED=23,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_SENT=24,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_RECD=25,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_ACK_SENT=26,
 NET_NF_CONNTRACK_COUNT=27,
 NET_NF_CONNTRACK_ICMPV6_TIMEOUT=28,
 NET_NF_CONNTRACK_FRAG6_TIMEOUT=29,
 NET_NF_CONNTRACK_FRAG6_LOW_THRESH=30,
 NET_NF_CONNTRACK_FRAG6_HIGH_THRESH=31,
 NET_NF_CONNTRACK_CHECKSUM=32,
};


enum
{

 NET_IPV4_FORWARD=8,
 NET_IPV4_DYNADDR=9,

 NET_IPV4_CONF=16,
 NET_IPV4_NEIGH=17,
 NET_IPV4_ROUTE=18,
 NET_IPV4_FIB_HASH=19,
 NET_IPV4_NETFILTER=20,

 NET_IPV4_TCP_TIMESTAMPS=33,
 NET_IPV4_TCP_WINDOW_SCALING=34,
 NET_IPV4_TCP_SACK=35,
 NET_IPV4_TCP_RETRANS_COLLAPSE=36,
 NET_IPV4_DEFAULT_TTL=37,
 NET_IPV4_AUTOCONFIG=38,
 NET_IPV4_NO_PMTU_DISC=39,
 NET_IPV4_TCP_SYN_RETRIES=40,
 NET_IPV4_IPFRAG_HIGH_THRESH=41,
 NET_IPV4_IPFRAG_LOW_THRESH=42,
 NET_IPV4_IPFRAG_TIME=43,
 NET_IPV4_TCP_MAX_KA_PROBES=44,
 NET_IPV4_TCP_KEEPALIVE_TIME=45,
 NET_IPV4_TCP_KEEPALIVE_PROBES=46,
 NET_IPV4_TCP_RETRIES1=47,
 NET_IPV4_TCP_RETRIES2=48,
 NET_IPV4_TCP_FIN_TIMEOUT=49,
 NET_IPV4_IP_MASQ_DEBUG=50,
 NET_TCP_SYNCOOKIES=51,
 NET_TCP_STDURG=52,
 NET_TCP_RFC1337=53,
 NET_TCP_SYN_TAILDROP=54,
 NET_TCP_MAX_SYN_BACKLOG=55,
 NET_IPV4_LOCAL_PORT_RANGE=56,
 NET_IPV4_ICMP_ECHO_IGNORE_ALL=57,
 NET_IPV4_ICMP_ECHO_IGNORE_BROADCASTS=58,
 NET_IPV4_ICMP_SOURCEQUENCH_RATE=59,
 NET_IPV4_ICMP_DESTUNREACH_RATE=60,
 NET_IPV4_ICMP_TIMEEXCEED_RATE=61,
 NET_IPV4_ICMP_PARAMPROB_RATE=62,
 NET_IPV4_ICMP_ECHOREPLY_RATE=63,
 NET_IPV4_ICMP_IGNORE_BOGUS_ERROR_RESPONSES=64,
 NET_IPV4_IGMP_MAX_MEMBERSHIPS=65,
 NET_TCP_TW_RECYCLE=66,
 NET_IPV4_ALWAYS_DEFRAG=67,
 NET_IPV4_TCP_KEEPALIVE_INTVL=68,
 NET_IPV4_INET_PEER_THRESHOLD=69,
 NET_IPV4_INET_PEER_MINTTL=70,
 NET_IPV4_INET_PEER_MAXTTL=71,
 NET_IPV4_INET_PEER_GC_MINTIME=72,
 NET_IPV4_INET_PEER_GC_MAXTIME=73,
 NET_TCP_ORPHAN_RETRIES=74,
 NET_TCP_ABORT_ON_OVERFLOW=75,
 NET_TCP_SYNACK_RETRIES=76,
 NET_TCP_MAX_ORPHANS=77,
 NET_TCP_MAX_TW_BUCKETS=78,
 NET_TCP_FACK=79,
 NET_TCP_REORDERING=80,
 NET_TCP_ECN=81,
 NET_TCP_DSACK=82,
 NET_TCP_MEM=83,
 NET_TCP_WMEM=84,
 NET_TCP_RMEM=85,
 NET_TCP_APP_WIN=86,
 NET_TCP_ADV_WIN_SCALE=87,
 NET_IPV4_NONLOCAL_BIND=88,
 NET_IPV4_ICMP_RATELIMIT=89,
 NET_IPV4_ICMP_RATEMASK=90,
 NET_TCP_TW_REUSE=91,
 NET_TCP_FRTO=92,
 NET_TCP_LOW_LATENCY=93,
 NET_IPV4_IPFRAG_SECRET_INTERVAL=94,
 NET_IPV4_IGMP_MAX_MSF=96,
 NET_TCP_NO_METRICS_SAVE=97,
 NET_TCP_DEFAULT_WIN_SCALE=105,
 NET_TCP_MODERATE_RCVBUF=106,
 NET_TCP_TSO_WIN_DIVISOR=107,
 NET_TCP_BIC_BETA=108,
 NET_IPV4_ICMP_ERRORS_USE_INBOUND_IFADDR=109,
 NET_TCP_CONG_CONTROL=110,
 NET_TCP_ABC=111,
 NET_IPV4_IPFRAG_MAX_DIST=112,
  NET_TCP_MTU_PROBING=113,
 NET_TCP_BASE_MSS=114,
 NET_IPV4_TCP_WORKAROUND_SIGNED_WINDOWS=115,
 NET_TCP_DMA_COPYBREAK=116,
 NET_TCP_SLOW_START_AFTER_IDLE=117,
 NET_CIPSOV4_CACHE_ENABLE=118,
 NET_CIPSOV4_CACHE_BUCKET_SIZE=119,
 NET_CIPSOV4_RBM_OPTFMT=120,
 NET_CIPSOV4_RBM_STRICTVALID=121,
 NET_TCP_AVAIL_CONG_CONTROL=122,
 NET_TCP_ALLOWED_CONG_CONTROL=123,
 NET_TCP_MAX_SSTHRESH=124,
 NET_TCP_FRTO_RESPONSE=125,
};

enum {
 NET_IPV4_ROUTE_FLUSH=1,
 NET_IPV4_ROUTE_MIN_DELAY=2,
 NET_IPV4_ROUTE_MAX_DELAY=3,
 NET_IPV4_ROUTE_GC_THRESH=4,
 NET_IPV4_ROUTE_MAX_SIZE=5,
 NET_IPV4_ROUTE_GC_MIN_INTERVAL=6,
 NET_IPV4_ROUTE_GC_TIMEOUT=7,
 NET_IPV4_ROUTE_GC_INTERVAL=8,
 NET_IPV4_ROUTE_REDIRECT_LOAD=9,
 NET_IPV4_ROUTE_REDIRECT_NUMBER=10,
 NET_IPV4_ROUTE_REDIRECT_SILENCE=11,
 NET_IPV4_ROUTE_ERROR_COST=12,
 NET_IPV4_ROUTE_ERROR_BURST=13,
 NET_IPV4_ROUTE_GC_ELASTICITY=14,
 NET_IPV4_ROUTE_MTU_EXPIRES=15,
 NET_IPV4_ROUTE_MIN_PMTU=16,
 NET_IPV4_ROUTE_MIN_ADVMSS=17,
 NET_IPV4_ROUTE_SECRET_INTERVAL=18,
 NET_IPV4_ROUTE_GC_MIN_INTERVAL_MS=19,
};

enum
{
 NET_PROTO_CONF_ALL=-2,
 NET_PROTO_CONF_DEFAULT=-3


};

enum
{
 NET_IPV4_CONF_FORWARDING=1,
 NET_IPV4_CONF_MC_FORWARDING=2,
 NET_IPV4_CONF_PROXY_ARP=3,
 NET_IPV4_CONF_ACCEPT_REDIRECTS=4,
 NET_IPV4_CONF_SECURE_REDIRECTS=5,
 NET_IPV4_CONF_SEND_REDIRECTS=6,
 NET_IPV4_CONF_SHARED_MEDIA=7,
 NET_IPV4_CONF_RP_FILTER=8,
 NET_IPV4_CONF_ACCEPT_SOURCE_ROUTE=9,
 NET_IPV4_CONF_BOOTP_RELAY=10,
 NET_IPV4_CONF_LOG_MARTIANS=11,
 NET_IPV4_CONF_TAG=12,
 NET_IPV4_CONF_ARPFILTER=13,
 NET_IPV4_CONF_MEDIUM_ID=14,
 NET_IPV4_CONF_NOXFRM=15,
 NET_IPV4_CONF_NOPOLICY=16,
 NET_IPV4_CONF_FORCE_IGMP_VERSION=17,
 NET_IPV4_CONF_ARP_ANNOUNCE=18,
 NET_IPV4_CONF_ARP_IGNORE=19,
 NET_IPV4_CONF_PROMOTE_SECONDARIES=20,
 NET_IPV4_CONF_ARP_ACCEPT=21,
 __NET_IPV4_CONF_MAX
};


enum
{
 NET_IPV4_NF_CONNTRACK_MAX=1,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_SYN_SENT=2,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_SYN_RECV=3,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_ESTABLISHED=4,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_FIN_WAIT=5,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_CLOSE_WAIT=6,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_LAST_ACK=7,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_TIME_WAIT=8,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_CLOSE=9,
 NET_IPV4_NF_CONNTRACK_UDP_TIMEOUT=10,
 NET_IPV4_NF_CONNTRACK_UDP_TIMEOUT_STREAM=11,
 NET_IPV4_NF_CONNTRACK_ICMP_TIMEOUT=12,
 NET_IPV4_NF_CONNTRACK_GENERIC_TIMEOUT=13,
 NET_IPV4_NF_CONNTRACK_BUCKETS=14,
 NET_IPV4_NF_CONNTRACK_LOG_INVALID=15,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_MAX_RETRANS=16,
 NET_IPV4_NF_CONNTRACK_TCP_LOOSE=17,
 NET_IPV4_NF_CONNTRACK_TCP_BE_LIBERAL=18,
 NET_IPV4_NF_CONNTRACK_TCP_MAX_RETRANS=19,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_CLOSED=20,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_WAIT=21,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_ECHOED=22,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_ESTABLISHED=23,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_SENT=24,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_RECD=25,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_ACK_SENT=26,
 NET_IPV4_NF_CONNTRACK_COUNT=27,
 NET_IPV4_NF_CONNTRACK_CHECKSUM=28,
};


enum {
 NET_IPV6_CONF=16,
 NET_IPV6_NEIGH=17,
 NET_IPV6_ROUTE=18,
 NET_IPV6_ICMP=19,
 NET_IPV6_BINDV6ONLY=20,
 NET_IPV6_IP6FRAG_HIGH_THRESH=21,
 NET_IPV6_IP6FRAG_LOW_THRESH=22,
 NET_IPV6_IP6FRAG_TIME=23,
 NET_IPV6_IP6FRAG_SECRET_INTERVAL=24,
 NET_IPV6_MLD_MAX_MSF=25,
};

enum {
 NET_IPV6_ROUTE_FLUSH=1,
 NET_IPV6_ROUTE_GC_THRESH=2,
 NET_IPV6_ROUTE_MAX_SIZE=3,
 NET_IPV6_ROUTE_GC_MIN_INTERVAL=4,
 NET_IPV6_ROUTE_GC_TIMEOUT=5,
 NET_IPV6_ROUTE_GC_INTERVAL=6,
 NET_IPV6_ROUTE_GC_ELASTICITY=7,
 NET_IPV6_ROUTE_MTU_EXPIRES=8,
 NET_IPV6_ROUTE_MIN_ADVMSS=9,
 NET_IPV6_ROUTE_GC_MIN_INTERVAL_MS=10
};

enum {
 NET_IPV6_FORWARDING=1,
 NET_IPV6_HOP_LIMIT=2,
 NET_IPV6_MTU=3,
 NET_IPV6_ACCEPT_RA=4,
 NET_IPV6_ACCEPT_REDIRECTS=5,
 NET_IPV6_AUTOCONF=6,
 NET_IPV6_DAD_TRANSMITS=7,
 NET_IPV6_RTR_SOLICITS=8,
 NET_IPV6_RTR_SOLICIT_INTERVAL=9,
 NET_IPV6_RTR_SOLICIT_DELAY=10,
 NET_IPV6_USE_TEMPADDR=11,
 NET_IPV6_TEMP_VALID_LFT=12,
 NET_IPV6_TEMP_PREFERED_LFT=13,
 NET_IPV6_REGEN_MAX_RETRY=14,
 NET_IPV6_MAX_DESYNC_FACTOR=15,
 NET_IPV6_MAX_ADDRESSES=16,
 NET_IPV6_FORCE_MLD_VERSION=17,
 NET_IPV6_ACCEPT_RA_DEFRTR=18,
 NET_IPV6_ACCEPT_RA_PINFO=19,
 NET_IPV6_ACCEPT_RA_RTR_PREF=20,
 NET_IPV6_RTR_PROBE_INTERVAL=21,
 NET_IPV6_ACCEPT_RA_RT_INFO_MAX_PLEN=22,
 NET_IPV6_PROXY_NDP=23,
 NET_IPV6_ACCEPT_SOURCE_ROUTE=25,
 __NET_IPV6_MAX
};


enum {
 NET_IPV6_ICMP_RATELIMIT=1
};


enum {
 NET_NEIGH_MCAST_SOLICIT=1,
 NET_NEIGH_UCAST_SOLICIT=2,
 NET_NEIGH_APP_SOLICIT=3,
 NET_NEIGH_RETRANS_TIME=4,
 NET_NEIGH_REACHABLE_TIME=5,
 NET_NEIGH_DELAY_PROBE_TIME=6,
 NET_NEIGH_GC_STALE_TIME=7,
 NET_NEIGH_UNRES_QLEN=8,
 NET_NEIGH_PROXY_QLEN=9,
 NET_NEIGH_ANYCAST_DELAY=10,
 NET_NEIGH_PROXY_DELAY=11,
 NET_NEIGH_LOCKTIME=12,
 NET_NEIGH_GC_INTERVAL=13,
 NET_NEIGH_GC_THRESH1=14,
 NET_NEIGH_GC_THRESH2=15,
 NET_NEIGH_GC_THRESH3=16,
 NET_NEIGH_RETRANS_TIME_MS=17,
 NET_NEIGH_REACHABLE_TIME_MS=18,
 __NET_NEIGH_MAX
};


enum {
 NET_DCCP_DEFAULT=1,
};


enum {
 NET_IPX_PPROP_BROADCASTING=1,
 NET_IPX_FORWARDING=2
};


enum {
 NET_LLC2=1,
 NET_LLC_STATION=2,
};


enum {
 NET_LLC2_TIMEOUT=1,
};


enum {
 NET_LLC_STATION_ACK_TIMEOUT=1,
};


enum {
 NET_LLC2_ACK_TIMEOUT=1,
 NET_LLC2_P_TIMEOUT=2,
 NET_LLC2_REJ_TIMEOUT=3,
 NET_LLC2_BUSY_TIMEOUT=4,
};


enum {
 NET_ATALK_AARP_EXPIRY_TIME=1,
 NET_ATALK_AARP_TICK_TIME=2,
 NET_ATALK_AARP_RETRANSMIT_LIMIT=3,
 NET_ATALK_AARP_RESOLVE_TIME=4
};



enum {
 NET_NETROM_DEFAULT_PATH_QUALITY=1,
 NET_NETROM_OBSOLESCENCE_COUNT_INITIALISER=2,
 NET_NETROM_NETWORK_TTL_INITIALISER=3,
 NET_NETROM_TRANSPORT_TIMEOUT=4,
 NET_NETROM_TRANSPORT_MAXIMUM_TRIES=5,
 NET_NETROM_TRANSPORT_ACKNOWLEDGE_DELAY=6,
 NET_NETROM_TRANSPORT_BUSY_DELAY=7,
 NET_NETROM_TRANSPORT_REQUESTED_WINDOW_SIZE=8,
 NET_NETROM_TRANSPORT_NO_ACTIVITY_TIMEOUT=9,
 NET_NETROM_ROUTING_CONTROL=10,
 NET_NETROM_LINK_FAILS_COUNT=11,
 NET_NETROM_RESET=12
};


enum {
 NET_AX25_IP_DEFAULT_MODE=1,
 NET_AX25_DEFAULT_MODE=2,
 NET_AX25_BACKOFF_TYPE=3,
 NET_AX25_CONNECT_MODE=4,
 NET_AX25_STANDARD_WINDOW=5,
 NET_AX25_EXTENDED_WINDOW=6,
 NET_AX25_T1_TIMEOUT=7,
 NET_AX25_T2_TIMEOUT=8,
 NET_AX25_T3_TIMEOUT=9,
 NET_AX25_IDLE_TIMEOUT=10,
 NET_AX25_N2=11,
 NET_AX25_PACLEN=12,
 NET_AX25_PROTOCOL=13,
 NET_AX25_DAMA_SLAVE_TIMEOUT=14
};


enum {
 NET_ROSE_RESTART_REQUEST_TIMEOUT=1,
 NET_ROSE_CALL_REQUEST_TIMEOUT=2,
 NET_ROSE_RESET_REQUEST_TIMEOUT=3,
 NET_ROSE_CLEAR_REQUEST_TIMEOUT=4,
 NET_ROSE_ACK_HOLD_BACK_TIMEOUT=5,
 NET_ROSE_ROUTING_CONTROL=6,
 NET_ROSE_LINK_FAIL_TIMEOUT=7,
 NET_ROSE_MAX_VCS=8,
 NET_ROSE_WINDOW_SIZE=9,
 NET_ROSE_NO_ACTIVITY_TIMEOUT=10
};


enum {
 NET_X25_RESTART_REQUEST_TIMEOUT=1,
 NET_X25_CALL_REQUEST_TIMEOUT=2,
 NET_X25_RESET_REQUEST_TIMEOUT=3,
 NET_X25_CLEAR_REQUEST_TIMEOUT=4,
 NET_X25_ACK_HOLD_BACK_TIMEOUT=5,
 NET_X25_FORWARD=6
};


enum
{
 NET_TR_RIF_TIMEOUT=1
};


enum {
 NET_DECNET_NODE_TYPE = 1,
 NET_DECNET_NODE_ADDRESS = 2,
 NET_DECNET_NODE_NAME = 3,
 NET_DECNET_DEFAULT_DEVICE = 4,
 NET_DECNET_TIME_WAIT = 5,
 NET_DECNET_DN_COUNT = 6,
 NET_DECNET_DI_COUNT = 7,
 NET_DECNET_DR_COUNT = 8,
 NET_DECNET_DST_GC_INTERVAL = 9,
 NET_DECNET_CONF = 10,
 NET_DECNET_NO_FC_MAX_CWND = 11,
 NET_DECNET_MEM = 12,
 NET_DECNET_RMEM = 13,
 NET_DECNET_WMEM = 14,
 NET_DECNET_DEBUG_LEVEL = 255
};


enum {
 NET_DECNET_CONF_LOOPBACK = -2,
 NET_DECNET_CONF_DDCMP = -3,
 NET_DECNET_CONF_PPP = -4,
 NET_DECNET_CONF_X25 = -5,
 NET_DECNET_CONF_GRE = -6,
 NET_DECNET_CONF_ETHER = -7


};


enum {
 NET_DECNET_CONF_DEV_PRIORITY = 1,
 NET_DECNET_CONF_DEV_T1 = 2,
 NET_DECNET_CONF_DEV_T2 = 3,
 NET_DECNET_CONF_DEV_T3 = 4,
 NET_DECNET_CONF_DEV_FORWARDING = 5,
 NET_DECNET_CONF_DEV_BLKSIZE = 6,
 NET_DECNET_CONF_DEV_STATE = 7
};


enum {
 NET_SCTP_RTO_INITIAL = 1,
 NET_SCTP_RTO_MIN = 2,
 NET_SCTP_RTO_MAX = 3,
 NET_SCTP_RTO_ALPHA = 4,
 NET_SCTP_RTO_BETA = 5,
 NET_SCTP_VALID_COOKIE_LIFE = 6,
 NET_SCTP_ASSOCIATION_MAX_RETRANS = 7,
 NET_SCTP_PATH_MAX_RETRANS = 8,
 NET_SCTP_MAX_INIT_RETRANSMITS = 9,
 NET_SCTP_HB_INTERVAL = 10,
 NET_SCTP_PRESERVE_ENABLE = 11,
 NET_SCTP_MAX_BURST = 12,
 NET_SCTP_ADDIP_ENABLE = 13,
 NET_SCTP_PRSCTP_ENABLE = 14,
 NET_SCTP_SNDBUF_POLICY = 15,
 NET_SCTP_SACK_TIMEOUT = 16,
 NET_SCTP_RCVBUF_POLICY = 17,
};


enum {
 NET_BRIDGE_NF_CALL_ARPTABLES = 1,
 NET_BRIDGE_NF_CALL_IPTABLES = 2,
 NET_BRIDGE_NF_CALL_IP6TABLES = 3,
 NET_BRIDGE_NF_FILTER_VLAN_TAGGED = 4,
 NET_BRIDGE_NF_FILTER_PPPOE_TAGGED = 5,
};


enum {
 NET_IRDA_DISCOVERY=1,
 NET_IRDA_DEVNAME=2,
 NET_IRDA_DEBUG=3,
 NET_IRDA_FAST_POLL=4,
 NET_IRDA_DISCOVERY_SLOTS=5,
 NET_IRDA_DISCOVERY_TIMEOUT=6,
 NET_IRDA_SLOT_TIMEOUT=7,
 NET_IRDA_MAX_BAUD_RATE=8,
 NET_IRDA_MIN_TX_TURN_TIME=9,
 NET_IRDA_MAX_TX_DATA_SIZE=10,
 NET_IRDA_MAX_TX_WINDOW=11,
 NET_IRDA_MAX_NOREPLY_TIME=12,
 NET_IRDA_WARN_NOREPLY_TIME=13,
 NET_IRDA_LAP_KEEPALIVE_TIME=14,
};



enum
{
 FS_NRINODE=1,
 FS_STATINODE=2,
 FS_MAXINODE=3,
 FS_NRDQUOT=4,
 FS_MAXDQUOT=5,
 FS_NRFILE=6,
 FS_MAXFILE=7,
 FS_DENTRY=8,
 FS_NRSUPER=9,
 FS_MAXSUPER=10,
 FS_OVERFLOWUID=11,
 FS_OVERFLOWGID=12,
 FS_LEASES=13,
 FS_DIR_NOTIFY=14,
 FS_LEASE_TIME=15,
 FS_DQSTATS=16,
 FS_XFS=17,
 FS_AIO_NR=18,
 FS_AIO_MAX_NR=19,
 FS_INOTIFY=20,
 FS_OCFS2=988,
};


enum {
 FS_DQ_LOOKUPS = 1,
 FS_DQ_DROPS = 2,
 FS_DQ_READS = 3,
 FS_DQ_WRITES = 4,
 FS_DQ_CACHE_HITS = 5,
 FS_DQ_ALLOCATED = 6,
 FS_DQ_FREE = 7,
 FS_DQ_SYNCS = 8,
 FS_DQ_WARNINGS = 9,
};




enum {
 DEV_CDROM=1,
 DEV_HWMON=2,
 DEV_PARPORT=3,
 DEV_RAID=4,
 DEV_MAC_HID=5,
 DEV_SCSI=6,
 DEV_IPMI=7,
};


enum {
 DEV_CDROM_INFO=1,
 DEV_CDROM_AUTOCLOSE=2,
 DEV_CDROM_AUTOEJECT=3,
 DEV_CDROM_DEBUG=4,
 DEV_CDROM_LOCK=5,
 DEV_CDROM_CHECK_MEDIA=6
};


enum {
 DEV_PARPORT_DEFAULT=-3
};


enum {
 DEV_RAID_SPEED_LIMIT_MIN=1,
 DEV_RAID_SPEED_LIMIT_MAX=2
};


enum {
 DEV_PARPORT_DEFAULT_TIMESLICE=1,
 DEV_PARPORT_DEFAULT_SPINTIME=2
};


enum {
 DEV_PARPORT_SPINTIME=1,
 DEV_PARPORT_BASE_ADDR=2,
 DEV_PARPORT_IRQ=3,
 DEV_PARPORT_DMA=4,
 DEV_PARPORT_MODES=5,
 DEV_PARPORT_DEVICES=6,
 DEV_PARPORT_AUTOPROBE=16
};


enum {
 DEV_PARPORT_DEVICES_ACTIVE=-3,
};


enum {
 DEV_PARPORT_DEVICE_TIMESLICE=1,
};


enum {
 DEV_MAC_HID_KEYBOARD_SENDS_LINUX_KEYCODES=1,
 DEV_MAC_HID_KEYBOARD_LOCK_KEYCODES=2,
 DEV_MAC_HID_MOUSE_BUTTON_EMULATION=3,
 DEV_MAC_HID_MOUSE_BUTTON2_KEYCODE=4,
 DEV_MAC_HID_MOUSE_BUTTON3_KEYCODE=5,
 DEV_MAC_HID_ADB_MOUSE_SENDS_KEYCODES=6
};


enum {
 DEV_SCSI_LOGGING_LEVEL=1,
};


enum {
 DEV_IPMI_POWEROFF_POWERCYCLE=1,
};


enum
{
 ABI_DEFHANDLER_COFF=1,
 ABI_DEFHANDLER_ELF=2,
 ABI_DEFHANDLER_LCALL7=3,
 ABI_DEFHANDLER_LIBCSO=4,
 ABI_TRACE=5,
 ABI_FAKE_UTSNAME=6,
};





struct ctl_table;
struct nsproxy;
struct ctl_table_root;

struct ctl_table_set {
 struct list_head list;
 struct ctl_table_set *parent;
 int (*is_seen)(struct ctl_table_set *);
};

extern void setup_sysctl_set(struct ctl_table_set *p,
 struct ctl_table_set *parent,
 int (*is_seen)(struct ctl_table_set *));

struct ctl_table_header;

extern void sysctl_head_get(struct ctl_table_header *);
extern void sysctl_head_put(struct ctl_table_header *);
extern int sysctl_is_seen(struct ctl_table_header *);
extern struct ctl_table_header *sysctl_head_grab(struct ctl_table_header *);
extern struct ctl_table_header *sysctl_head_next(struct ctl_table_header *prev);
extern struct ctl_table_header *__sysctl_head_next(struct nsproxy *namespaces,
      struct ctl_table_header *prev);
extern void sysctl_head_finish(struct ctl_table_header *prev);
extern int sysctl_perm(struct ctl_table_root *root,
  struct ctl_table *table, int op);

typedef struct ctl_table ctl_table;

typedef int ctl_handler (struct ctl_table *table, int *name, int nlen,
    void *oldval, size_t *oldlenp,
    void *newval, size_t newlen);

typedef int proc_handler (struct ctl_table *ctl, int write, struct file * filp,
     void *buffer, size_t *lenp, loff_t *ppos);

extern int proc_dostring(struct ctl_table *, int, struct file *,
    void *, size_t *, loff_t *);
extern int proc_dointvec(struct ctl_table *, int, struct file *,
    void *, size_t *, loff_t *);
extern int proc_dointvec_minmax(struct ctl_table *, int, struct file *,
    void *, size_t *, loff_t *);
extern int proc_dointvec_jiffies(struct ctl_table *, int, struct file *,
     void *, size_t *, loff_t *);
extern int proc_dointvec_userhz_jiffies(struct ctl_table *, int, struct file *,
     void *, size_t *, loff_t *);
extern int proc_dointvec_ms_jiffies(struct ctl_table *, int, struct file *,
        void *, size_t *, loff_t *);
extern int proc_doulongvec_minmax(struct ctl_table *, int, struct file *,
      void *, size_t *, loff_t *);
extern int proc_doulongvec_ms_jiffies_minmax(struct ctl_table *table, int,
          struct file *, void *, size_t *, loff_t *);

extern int do_sysctl (int *name, int nlen,
        void *oldval, size_t *oldlenp,
        void *newval, size_t newlen);

extern ctl_handler sysctl_data;
extern ctl_handler sysctl_string;
extern ctl_handler sysctl_intvec;
extern ctl_handler sysctl_jiffies;
extern ctl_handler sysctl_ms_jiffies;
struct ctl_table
{
 int ctl_name;
 char *procname;
 void *data;
 int maxlen;
 mode_t mode;
 struct ctl_table *child;
 struct ctl_table *parent;
 proc_handler *proc_handler;
 ctl_handler *strategy;
 void *extra1;
 void *extra2;
};

struct ctl_table_root {
 struct list_head root_list;
 struct ctl_table_set default_set;
 struct ctl_table_set *(*lookup)(struct ctl_table_root *root,
        struct nsproxy *namespaces);
 int (*permissions)(struct ctl_table_root *root,
   struct nsproxy *namespaces, struct ctl_table *table);
};



struct ctl_table_header
{
 struct ctl_table *ctl_table;
 struct list_head ctl_entry;
 int used;
 int count;
 struct completion *unregistering;
 struct ctl_table *ctl_table_arg;
 struct ctl_table_root *root;
 struct ctl_table_set *set;
 struct ctl_table *attached_by;
 struct ctl_table *attached_to;
 struct ctl_table_header *parent;
};


struct ctl_path {
 char *procname;
 int ctl_name;
};

void register_sysctl_root(struct ctl_table_root *root);
struct ctl_table_header *__register_sysctl_paths(
 struct ctl_table_root *root, struct nsproxy *namespaces,
 struct ctl_path *path, struct ctl_table *table);
struct ctl_table_header *register_sysctl_table(struct ctl_table * table);
struct ctl_table_header *register_sysctl_paths( struct ctl_path *path,
      struct ctl_table *table);

void unregister_sysctl_table(struct ctl_table_header * table);
int sysctl_check_table(struct nsproxy *namespaces, struct ctl_table *table);
struct taskstats {





 __u16 version;
 __u32 ac_exitcode;




 __u8 ac_flag;
 __u8 ac_nice;
 __u64 cpu_count __attribute__((aligned(8)));
 __u64 cpu_delay_total;






 __u64 blkio_count;
 __u64 blkio_delay_total;


 __u64 swapin_count;
 __u64 swapin_delay_total;







 __u64 cpu_run_real_total;







 __u64 cpu_run_virtual_total;




 char ac_comm[32];
 __u8 ac_sched __attribute__((aligned(8)));

 __u8 ac_pad[3];
 __u32 ac_uid __attribute__((aligned(8)));

 __u32 ac_gid;
 __u32 ac_pid;
 __u32 ac_ppid;
 __u32 ac_btime;
 __u64 ac_etime __attribute__((aligned(8)));

 __u64 ac_utime;
 __u64 ac_stime;
 __u64 ac_minflt;
 __u64 ac_majflt;
 __u64 coremem;



 __u64 virtmem;




 __u64 hiwater_rss;
 __u64 hiwater_vm;


 __u64 read_char;
 __u64 write_char;
 __u64 read_syscalls;
 __u64 write_syscalls;




 __u64 read_bytes;
 __u64 write_bytes;
 __u64 cancelled_write_bytes;

 __u64 nvcsw;
 __u64 nivcsw;


 __u64 ac_utimescaled;
 __u64 ac_stimescaled;
 __u64 cpu_scaled_run_real_total;


 __u64 freepages_count;
 __u64 freepages_delay_total;
};
enum {
 TASKSTATS_CMD_UNSPEC = 0,
 TASKSTATS_CMD_GET,
 TASKSTATS_CMD_NEW,
 __TASKSTATS_CMD_MAX,
};



enum {
 TASKSTATS_TYPE_UNSPEC = 0,
 TASKSTATS_TYPE_PID,
 TASKSTATS_TYPE_TGID,
 TASKSTATS_TYPE_STATS,
 TASKSTATS_TYPE_AGGR_PID,
 TASKSTATS_TYPE_AGGR_TGID,
 __TASKSTATS_TYPE_MAX,
};



enum {
 TASKSTATS_CMD_ATTR_UNSPEC = 0,
 TASKSTATS_CMD_ATTR_PID,
 TASKSTATS_CMD_ATTR_TGID,
 TASKSTATS_CMD_ATTR_REGISTER_CPUMASK,
 TASKSTATS_CMD_ATTR_DEREGISTER_CPUMASK,
 __TASKSTATS_CMD_ATTR_MAX,
};










struct __kernel_sockaddr_storage {
 unsigned short ss_family;

 char __data[128 - sizeof(unsigned short)];


} __attribute__ ((aligned((__alignof__ (struct sockaddr *)))));











struct seq_file;
extern void socket_seq_show(struct seq_file *seq);


typedef unsigned short sa_family_t;





struct sockaddr {
 sa_family_t sa_family;
 char sa_data[14];
};

struct linger {
 int l_onoff;
 int l_linger;
};
struct msghdr {
 void * msg_name;
 int msg_namelen;
 struct iovec * msg_iov;
 __kernel_size_t msg_iovlen;
 void * msg_control;
 __kernel_size_t msg_controllen;
 unsigned msg_flags;
};







struct cmsghdr {
 __kernel_size_t cmsg_len;
        int cmsg_level;
        int cmsg_type;
};
static inline struct cmsghdr * __cmsg_nxthdr(void *__ctl, __kernel_size_t __size,
            struct cmsghdr *__cmsg)
{
 struct cmsghdr * __ptr;

 __ptr = (struct cmsghdr*)(((unsigned char *) __cmsg) + ( ((__cmsg->cmsg_len)+sizeof(long)-1) & ~(sizeof(long)-1) ));
 if ((unsigned long)((char*)(__ptr+1) - (char *) __ctl) > __size)
  return (struct cmsghdr *)0;

 return __ptr;
}

static inline struct cmsghdr * cmsg_nxthdr (struct msghdr *__msg, struct cmsghdr *__cmsg)
{
 return __cmsg_nxthdr(__msg->msg_control, __msg->msg_controllen, __cmsg);
}







struct ucred {
 __u32 pid;
 __u32 uid;
 __u32 gid;
};
extern int memcpy_fromiovec(unsigned char *kdata, struct iovec *iov, int len);
extern int memcpy_fromiovecend(unsigned char *kdata, struct iovec *iov,
    int offset, int len);
extern int csum_partial_copy_fromiovecend(unsigned char *kdata,
       struct iovec *iov,
       int offset,
       unsigned int len, __wsum *csump);

extern int verify_iovec(struct msghdr *m, struct iovec *iov, struct sockaddr *address, int mode);
extern int memcpy_toiovec(struct iovec *v, unsigned char *kdata, int len);
extern int move_addr_to_user(struct sockaddr *kaddr, int klen, void *uaddr, int *ulen);
extern int move_addr_to_kernel(void *uaddr, int ulen, struct sockaddr *kaddr);
extern int put_cmsg(struct msghdr*, int level, int type, int len, void *data);
struct net;

struct sockaddr_nl
{
 sa_family_t nl_family;
 unsigned short nl_pad;
 __u32 nl_pid;
        __u32 nl_groups;
};

struct nlmsghdr
{
 __u32 nlmsg_len;
 __u16 nlmsg_type;
 __u16 nlmsg_flags;
 __u32 nlmsg_seq;
 __u32 nlmsg_pid;
};
struct nlmsgerr
{
 int error;
 struct nlmsghdr msg;
};





struct nl_pktinfo
{
 __u32 group;
};



enum {
 NETLINK_UNCONNECTED = 0,
 NETLINK_CONNECTED,
};
struct nlattr
{
 __u16 nla_len;
 __u16 nla_type;
};


typedef enum {
 SS_FREE = 0,
 SS_UNCONNECTED,
 SS_CONNECTING,
 SS_CONNECTED,
 SS_DISCONNECTING
} socket_state;








extern unsigned int __invalid_size_argument_for_IOC;
struct rand_pool_info {
 int entropy_count;
 int buf_size;
 __u32 buf[0];
};





extern void rand_initialize_irq(int irq);

extern void add_input_randomness(unsigned int type, unsigned int code,
     unsigned int value);
extern void add_interrupt_randomness(int irq);

extern void get_random_bytes(void *buf, int nbytes);
void generate_random_uuid(unsigned char uuid_out[16]);

extern __u32 secure_ip_id(__be32 daddr);
extern u32 secure_ipv4_port_ephemeral(__be32 saddr, __be32 daddr, __be16 dport);
extern u32 secure_ipv6_port_ephemeral( __be32 *saddr, __be32 *daddr,
          __be16 dport);
extern __u32 secure_tcp_sequence_number(__be32 saddr, __be32 daddr,
     __be16 sport, __be16 dport);
extern __u32 secure_tcpv6_sequence_number(__be32 *saddr, __be32 *daddr,
       __be16 sport, __be16 dport);
extern u64 secure_dccp_sequence_number(__be32 saddr, __be32 daddr,
           __be16 sport, __be16 dport);


extern struct file_operations random_fops, urandom_fops;


unsigned int get_random_int(void);
unsigned long randomize_range(unsigned long start, unsigned long end, unsigned long len);

u32 random32(void);
void srandom32(u32 seed);




struct flock {
 short l_type;
 short l_whence;
 off_t l_start;
 off_t l_len;
 pid_t l_pid;

};

struct poll_table_struct;
struct pipe_inode_info;
struct inode;
struct net;
enum sock_type {
 SOCK_STREAM = 1,
 SOCK_DGRAM = 2,
 SOCK_RAW = 3,
 SOCK_RDM = 4,
 SOCK_SEQPACKET = 5,
 SOCK_DCCP = 6,
 SOCK_PACKET = 10,
};
enum sock_shutdown_cmd {
 SHUT_RD = 0,
 SHUT_WR = 1,
 SHUT_RDWR = 2,
};
struct socket {
 socket_state state;
 short type;
 unsigned long flags;
 struct proto_ops *ops;
 struct fasync_struct *fasync_list;
 struct file *file;
 struct sock *sk;
 wait_queue_head_t wait;
};

struct vm_area_struct;
struct page;
struct kiocb;
struct sockaddr;
struct msghdr;
struct module;

struct proto_ops {
 int family;
 struct module *owner;
 int (*release) (struct socket *sock);
 int (*bind) (struct socket *sock,
          struct sockaddr *myaddr,
          int sockaddr_len);
 int (*connect) (struct socket *sock,
          struct sockaddr *vaddr,
          int sockaddr_len, int flags);
 int (*socketpair)(struct socket *sock1,
          struct socket *sock2);
 int (*accept) (struct socket *sock,
          struct socket *newsock, int flags);
 int (*getname) (struct socket *sock,
          struct sockaddr *addr,
          int *sockaddr_len, int peer);
 unsigned int (*poll) (struct file *file, struct socket *sock,
          struct poll_table_struct *wait);
 int (*ioctl) (struct socket *sock, unsigned int cmd,
          unsigned long arg);
 int (*compat_ioctl) (struct socket *sock, unsigned int cmd,
          unsigned long arg);
 int (*listen) (struct socket *sock, int len);
 int (*shutdown) (struct socket *sock, int flags);
 int (*setsockopt)(struct socket *sock, int level,
          int optname, char *optval, int optlen);
 int (*getsockopt)(struct socket *sock, int level,
          int optname, char *optval, int *optlen);
 int (*compat_setsockopt)(struct socket *sock, int level,
          int optname, char *optval, int optlen);
 int (*compat_getsockopt)(struct socket *sock, int level,
          int optname, char *optval, int *optlen);
 int (*sendmsg) (struct kiocb *iocb, struct socket *sock,
          struct msghdr *m, size_t total_len);
 int (*recvmsg) (struct kiocb *iocb, struct socket *sock,
          struct msghdr *m, size_t total_len,
          int flags);
 int (*mmap) (struct file *file, struct socket *sock,
          struct vm_area_struct * vma);
 ssize_t (*sendpage) (struct socket *sock, struct page *page,
          int offset, size_t size, int flags);
 ssize_t (*splice_read)(struct socket *sock, loff_t *ppos,
           struct pipe_inode_info *pipe, size_t len, unsigned int flags);
};

struct net_proto_family {
 int family;
 int (*create)(struct net *net, struct socket *sock, int protocol);
 struct module *owner;
};

struct iovec;
struct kvec;

enum {
 SOCK_WAKE_IO,
 SOCK_WAKE_WAITD,
 SOCK_WAKE_SPACE,
 SOCK_WAKE_URG,
};

extern int sock_wake_async(struct socket *sk, int how, int band);
extern int sock_register( struct net_proto_family *fam);
extern void sock_unregister(int family);
extern int sock_create(int family, int type, int proto,
     struct socket **res);
extern int sock_create_kern(int family, int type, int proto,
          struct socket **res);
extern int sock_create_lite(int family, int type, int proto,
          struct socket **res);
extern void sock_release(struct socket *sock);
extern int sock_sendmsg(struct socket *sock, struct msghdr *msg,
      size_t len);
extern int sock_recvmsg(struct socket *sock, struct msghdr *msg,
      size_t size, int flags);
extern int sock_map_fd(struct socket *sock, int flags);
extern struct socket *sockfd_lookup(int fd, int *err);

extern int net_ratelimit(void);
extern long do_accept(int fd, struct sockaddr *upeer_sockaddr,
          int *upeer_addrlen, int flags);




extern int kernel_sendmsg(struct socket *sock, struct msghdr *msg,
        struct kvec *vec, size_t num, size_t len);
extern int kernel_recvmsg(struct socket *sock, struct msghdr *msg,
        struct kvec *vec, size_t num,
        size_t len, int flags);

extern int kernel_bind(struct socket *sock, struct sockaddr *addr,
         int addrlen);
extern int kernel_listen(struct socket *sock, int backlog);
extern int kernel_accept(struct socket *sock, struct socket **newsock,
    int flags);
extern int kernel_connect(struct socket *sock, struct sockaddr *addr,
     int addrlen, int flags);
extern int kernel_getsockname(struct socket *sock, struct sockaddr *addr,
         int *addrlen);
extern int kernel_getpeername(struct socket *sock, struct sockaddr *addr,
         int *addrlen);
extern int kernel_getsockopt(struct socket *sock, int level, int optname,
        char *optval, int *optlen);
extern int kernel_setsockopt(struct socket *sock, int level, int optname,
        char *optval, int optlen);
extern int kernel_sendpage(struct socket *sock, struct page *page, int offset,
      size_t size, int flags);
extern int kernel_sock_ioctl(struct socket *sock, int cmd, unsigned long arg);
extern int kernel_sock_shutdown(struct socket *sock,
    enum sock_shutdown_cmd how);
extern struct ratelimit_state net_ratelimit_state;











struct stat {
 unsigned long st_dev;
 unsigned long st_ino;
 unsigned long st_nlink;

 unsigned int st_mode;
 unsigned int st_uid;
 unsigned int st_gid;
 unsigned int __pad0;
 unsigned long st_rdev;
 long st_size;
 long st_blksize;
 long st_blocks;

 unsigned long st_atime;
 unsigned long st_atime_nsec;
 unsigned long st_mtime;
 unsigned long st_mtime_nsec;
 unsigned long st_ctime;
 unsigned long st_ctime_nsec;
 long __unused[3];
};



struct __old_kernel_stat {
 unsigned short st_dev;
 unsigned short st_ino;
 unsigned short st_mode;
 unsigned short st_nlink;
 unsigned short st_uid;
 unsigned short st_gid;
 unsigned short st_rdev;






 unsigned int st_size;
 unsigned int st_atime;
 unsigned int st_mtime;
 unsigned int st_ctime;

};
struct kstat {
 u64 ino;
 dev_t dev;
 umode_t mode;
 unsigned int nlink;
 uid_t uid;
 gid_t gid;
 dev_t rdev;
 loff_t size;
 struct timespec atime;
 struct timespec mtime;
 struct timespec ctime;
 unsigned long blksize;
 unsigned long long blocks;
};


extern int request_module( char * name, ...) __attribute__ ((format (printf, 1, 2)));







struct key;
struct file;
struct subprocess_info;


struct subprocess_info *call_usermodehelper_setup(char *path, char **argv,
        char **envp, gfp_t gfp_mask);


void call_usermodehelper_setkeys(struct subprocess_info *info,
     struct key *session_keyring);
int call_usermodehelper_stdinpipe(struct subprocess_info *sub_info,
      struct file **filp);
void call_usermodehelper_setcleanup(struct subprocess_info *info,
        void (*cleanup)(char **argv, char **envp));

enum umh_wait {
 UMH_NO_WAIT = -1,
 UMH_WAIT_EXEC = 0,
 UMH_WAIT_PROC = 1,
};


int call_usermodehelper_exec(struct subprocess_info *info, enum umh_wait wait);



void call_usermodehelper_freeinfo(struct subprocess_info *info);

static inline int
call_usermodehelper(char *path, char **argv, char **envp, enum umh_wait wait)
{
 struct subprocess_info *info;
 gfp_t gfp_mask = (wait == UMH_NO_WAIT) ? ((( gfp_t)0x20u)) : ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u));

 info = call_usermodehelper_setup(path, argv, envp, gfp_mask);
 if (info == 0)
  return -12;
 return call_usermodehelper_exec(info, wait);
}

static inline int
call_usermodehelper_keys(char *path, char **argv, char **envp,
    struct key *session_keyring, enum umh_wait wait)
{
 struct subprocess_info *info;
 gfp_t gfp_mask = (wait == UMH_NO_WAIT) ? ((( gfp_t)0x20u)) : ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u));

 info = call_usermodehelper_setup(path, argv, envp, gfp_mask);
 if (info == 0)
  return -12;

 call_usermodehelper_setkeys(info, session_keyring);
 return call_usermodehelper_exec(info, wait);
}

extern void usermodehelper_init(void);

struct file;
extern int call_usermodehelper_pipe(char *path, char *argv[], char *envp[],
        struct file **filp);








struct user_i387_struct {
 unsigned short cwd;
 unsigned short swd;
 unsigned short twd;

 unsigned short fop;
 __u64 rip;
 __u64 rdp;
 __u32 mxcsr;
 __u32 mxcsr_mask;
 __u32 st_space[32];
 __u32 xmm_space[64];
 __u32 padding[24];
};




struct user_regs_struct {
 unsigned long r15;
 unsigned long r14;
 unsigned long r13;
 unsigned long r12;
 unsigned long bp;
 unsigned long bx;
 unsigned long r11;
 unsigned long r10;
 unsigned long r9;
 unsigned long r8;
 unsigned long ax;
 unsigned long cx;
 unsigned long dx;
 unsigned long si;
 unsigned long di;
 unsigned long orig_ax;
 unsigned long ip;
 unsigned long cs;
 unsigned long flags;
 unsigned long sp;
 unsigned long ss;
 unsigned long fs_base;
 unsigned long gs_base;
 unsigned long ds;
 unsigned long es;
 unsigned long fs;
 unsigned long gs;
};





struct user {


  struct user_regs_struct regs;

  int u_fpvalid;

  int pad0;
  struct user_i387_struct i387;

  unsigned long int u_tsize;
  unsigned long int u_dsize;
  unsigned long int u_ssize;
  unsigned long start_code;
  unsigned long start_stack;



  long int signal;
  int reserved;
  int pad1;
  unsigned long u_ar0;

  struct user_i387_struct *u_fpstate;
  unsigned long magic;
  char u_comm[32];
  unsigned long u_debugreg[8];
  unsigned long error_code;
  unsigned long fault_address;
};


typedef unsigned long elf_greg_t;


typedef elf_greg_t elf_gregset_t[(sizeof(struct user_regs_struct) / sizeof(elf_greg_t))];

typedef struct user_i387_struct elf_fpregset_t;




extern char VDSO64_PRELINK[];
extern char VDSO32_PRELINK[];
extern void* __kernel_sigreturn;
extern void* __kernel_rt_sigreturn;





extern char vdso32_int80_start, vdso32_int80_end;
extern char vdso32_syscall_start, vdso32_syscall_end;
extern char vdso32_sysenter_start, vdso32_sysenter_end;

extern unsigned int vdso_enabled;
static inline void start_ia32_thread(struct pt_regs *regs, u32 ip, u32 sp)
{
 asm ("movl %0,%%fs" :: "r" (0));
 asm ("movl %0,%%es; movl %0,%%ds" : : "r" ((5* 8 + 3)));
 load_gs_index(0);
 regs->ip = ip;
 regs->sp = sp;
 regs->flags = 0x00000200;
 regs->cs = (4 * 8 + 3);
 regs->ss = (5* 8 + 3);
}

static inline void elf_common_init(struct thread_struct *t,
       struct pt_regs *regs, u16 ds)
{
 regs->ax = regs->bx = regs->cx = regs->dx = 0;
 regs->si = regs->di = regs->bp = 0;
 regs->r8 = regs->r9 = regs->r10 = regs->r11 = 0;
 regs->r12 = regs->r13 = regs->r14 = regs->r15 = 0;
 t->fs = t->gs = 0;
 t->fsindex = t->gsindex = 0;
 t->ds = t->es = ds;
}
extern void set_personality_64bit(void);
extern unsigned int sysctl_vsyscall32;
extern int force_personality32;
struct task_struct;
struct linux_binprm;


extern int arch_setup_additional_pages(struct linux_binprm *bprm,
           int executable_stack);

extern int syscall32_setup_pages(struct linux_binprm *, int exstack);


extern unsigned long arch_randomize_brk(struct mm_struct *mm);


struct file;
typedef __u32 Elf32_Addr;
typedef __u16 Elf32_Half;
typedef __u32 Elf32_Off;
typedef __s32 Elf32_Sword;
typedef __u32 Elf32_Word;


typedef __u64 Elf64_Addr;
typedef __u16 Elf64_Half;
typedef __s16 Elf64_SHalf;
typedef __u64 Elf64_Off;
typedef __s32 Elf64_Sword;
typedef __u32 Elf64_Word;
typedef __u64 Elf64_Xword;
typedef __s64 Elf64_Sxword;
typedef struct dynamic{
  Elf32_Sword d_tag;
  union{
    Elf32_Sword d_val;
    Elf32_Addr d_ptr;
  } d_un;
} Elf32_Dyn;

typedef struct {
  Elf64_Sxword d_tag;
  union {
    Elf64_Xword d_val;
    Elf64_Addr d_ptr;
  } d_un;
} Elf64_Dyn;
typedef struct elf32_rel {
  Elf32_Addr r_offset;
  Elf32_Word r_info;
} Elf32_Rel;

typedef struct elf64_rel {
  Elf64_Addr r_offset;
  Elf64_Xword r_info;
} Elf64_Rel;

typedef struct elf32_rela{
  Elf32_Addr r_offset;
  Elf32_Word r_info;
  Elf32_Sword r_addend;
} Elf32_Rela;

typedef struct elf64_rela {
  Elf64_Addr r_offset;
  Elf64_Xword r_info;
  Elf64_Sxword r_addend;
} Elf64_Rela;

typedef struct elf32_sym{
  Elf32_Word st_name;
  Elf32_Addr st_value;
  Elf32_Word st_size;
  unsigned char st_info;
  unsigned char st_other;
  Elf32_Half st_shndx;
} Elf32_Sym;

typedef struct elf64_sym {
  Elf64_Word st_name;
  unsigned char st_info;
  unsigned char st_other;
  Elf64_Half st_shndx;
  Elf64_Addr st_value;
  Elf64_Xword st_size;
} Elf64_Sym;




typedef struct elf32_hdr{
  unsigned char e_ident[16];
  Elf32_Half e_type;
  Elf32_Half e_machine;
  Elf32_Word e_version;
  Elf32_Addr e_entry;
  Elf32_Off e_phoff;
  Elf32_Off e_shoff;
  Elf32_Word e_flags;
  Elf32_Half e_ehsize;
  Elf32_Half e_phentsize;
  Elf32_Half e_phnum;
  Elf32_Half e_shentsize;
  Elf32_Half e_shnum;
  Elf32_Half e_shstrndx;
} Elf32_Ehdr;

typedef struct elf64_hdr {
  unsigned char e_ident[16];
  Elf64_Half e_type;
  Elf64_Half e_machine;
  Elf64_Word e_version;
  Elf64_Addr e_entry;
  Elf64_Off e_phoff;
  Elf64_Off e_shoff;
  Elf64_Word e_flags;
  Elf64_Half e_ehsize;
  Elf64_Half e_phentsize;
  Elf64_Half e_phnum;
  Elf64_Half e_shentsize;
  Elf64_Half e_shnum;
  Elf64_Half e_shstrndx;
} Elf64_Ehdr;







typedef struct elf32_phdr{
  Elf32_Word p_type;
  Elf32_Off p_offset;
  Elf32_Addr p_vaddr;
  Elf32_Addr p_paddr;
  Elf32_Word p_filesz;
  Elf32_Word p_memsz;
  Elf32_Word p_flags;
  Elf32_Word p_align;
} Elf32_Phdr;

typedef struct elf64_phdr {
  Elf64_Word p_type;
  Elf64_Word p_flags;
  Elf64_Off p_offset;
  Elf64_Addr p_vaddr;
  Elf64_Addr p_paddr;
  Elf64_Xword p_filesz;
  Elf64_Xword p_memsz;
  Elf64_Xword p_align;
} Elf64_Phdr;
typedef struct {
  Elf32_Word sh_name;
  Elf32_Word sh_type;
  Elf32_Word sh_flags;
  Elf32_Addr sh_addr;
  Elf32_Off sh_offset;
  Elf32_Word sh_size;
  Elf32_Word sh_link;
  Elf32_Word sh_info;
  Elf32_Word sh_addralign;
  Elf32_Word sh_entsize;
} Elf32_Shdr;

typedef struct elf64_shdr {
  Elf64_Word sh_name;
  Elf64_Word sh_type;
  Elf64_Xword sh_flags;
  Elf64_Addr sh_addr;
  Elf64_Off sh_offset;
  Elf64_Xword sh_size;
  Elf64_Word sh_link;
  Elf64_Word sh_info;
  Elf64_Xword sh_addralign;
  Elf64_Xword sh_entsize;
} Elf64_Shdr;
typedef struct elf32_note {
  Elf32_Word n_namesz;
  Elf32_Word n_descsz;
  Elf32_Word n_type;
} Elf32_Nhdr;


typedef struct elf64_note {
  Elf64_Word n_namesz;
  Elf64_Word n_descsz;
  Elf64_Word n_type;
} Elf64_Nhdr;
extern Elf64_Dyn _DYNAMIC [];
static inline int elf_coredump_extra_notes_size(void) { return 0; }
static inline int elf_coredump_extra_notes_write(struct file *file,
   loff_t *foffset) { return 0; }


struct kernel_param;


typedef int (*param_set_fn)( char *val, struct kernel_param *kp);

typedef int (*param_get_fn)(char *buffer, struct kernel_param *kp);

struct kernel_param {
 char *name;
 unsigned int perm;
 param_set_fn set;
 param_get_fn get;
 union {
  void *arg;
  struct kparam_string *str;
  struct kparam_array *arr;
 };
};


struct kparam_string {
 unsigned int maxlen;
 char *string;
};


struct kparam_array
{
 unsigned int max;
 unsigned int *num;
 param_set_fn set;
 param_get_fn get;
 unsigned int elemsize;
 void *elem;
};
extern int parse_args( char *name,
        char *args,
        struct kernel_param *params,
        unsigned num,
        int (*unknown)(char *param, char *val));







extern int param_set_byte( char *val, struct kernel_param *kp);
extern int param_get_byte(char *buffer, struct kernel_param *kp);


extern int param_set_short( char *val, struct kernel_param *kp);
extern int param_get_short(char *buffer, struct kernel_param *kp);


extern int param_set_ushort( char *val, struct kernel_param *kp);
extern int param_get_ushort(char *buffer, struct kernel_param *kp);


extern int param_set_int( char *val, struct kernel_param *kp);
extern int param_get_int(char *buffer, struct kernel_param *kp);


extern int param_set_uint( char *val, struct kernel_param *kp);
extern int param_get_uint(char *buffer, struct kernel_param *kp);


extern int param_set_long( char *val, struct kernel_param *kp);
extern int param_get_long(char *buffer, struct kernel_param *kp);


extern int param_set_ulong( char *val, struct kernel_param *kp);
extern int param_get_ulong(char *buffer, struct kernel_param *kp);


extern int param_set_charp( char *val, struct kernel_param *kp);
extern int param_get_charp(char *buffer, struct kernel_param *kp);


extern int param_set_bool( char *val, struct kernel_param *kp);
extern int param_get_bool(char *buffer, struct kernel_param *kp);


extern int param_set_invbool( char *val, struct kernel_param *kp);
extern int param_get_invbool(char *buffer, struct kernel_param *kp);
extern int param_array_set( char *val, struct kernel_param *kp);
extern int param_array_get(char *buffer, struct kernel_param *kp);

extern int param_set_copystring( char *val, struct kernel_param *kp);
extern int param_get_string(char *buffer, struct kernel_param *kp);



struct module;


extern int module_param_sysfs_setup(struct module *mod,
        struct kernel_param *kparam,
        unsigned int num_params);

extern void module_param_sysfs_remove(struct module *mod);
struct module;
struct marker;
typedef void marker_probe_func(void *probe_private, void *call_private,
  char *fmt, va_list *args);

struct marker_probe_closure {
 marker_probe_func *func;
 void *probe_private;
};

struct marker {
 char *name;
 char *format;


 char state;
 char ptype;

 void (*call)( struct marker *mdata, void *call_private, ...);
 struct marker_probe_closure single;
 struct marker_probe_closure *multi;
} __attribute__((aligned(8)));
extern void marker_update_probe_range(struct marker *begin,
 struct marker *end);
static inline void __attribute__((format(printf,1,2))) ___mark_check_format( char *fmt, ...)
{
}







extern marker_probe_func __mark_empty_function;

extern void marker_probe_cb( struct marker *mdata,
 void *call_private, ...);
extern void marker_probe_cb_noarg( struct marker *mdata,
 void *call_private, ...);





extern int marker_probe_register( char *name, char *format,
    marker_probe_func *probe, void *probe_private);




extern int marker_probe_unregister( char *name,
 marker_probe_func *probe, void *probe_private);



extern int marker_probe_unregister_private_data(marker_probe_func *probe,
 void *probe_private);

extern void *marker_get_private_data( char *name, marker_probe_func *probe,
 int num);








typedef struct {
 atomic_long_t a;
} local_t;






static inline void local_inc(local_t *l)
{
 asm (" " "incq" " " "%0"
       : "+m" (l->a.counter));
}

static inline void local_dec(local_t *l)
{
 asm (" " "decq" " " "%0"
       : "+m" (l->a.counter));
}

static inline void local_add(long i, local_t *l)
{
 asm (" " "addq" " " "%1,%0"
       : "+m" (l->a.counter)
       : "ir" (i));
}

static inline void local_sub(long i, local_t *l)
{
 asm (" " "subq" " " "%1,%0"
       : "+m" (l->a.counter)
       : "ir" (i));
}
static inline int local_sub_and_test(long i, local_t *l)
{
 unsigned char c;

 asm (" " "subq" " " "%2,%0; sete %1"
       : "+m" (l->a.counter), "=qm" (c)
       : "ir" (i) : "memory");
 return c;
}
static inline int local_dec_and_test(local_t *l)
{
 unsigned char c;

 asm (" " "decq" " " "%0; sete %1"
       : "+m" (l->a.counter), "=qm" (c)
       : : "memory");
 return c != 0;
}
static inline int local_inc_and_test(local_t *l)
{
 unsigned char c;

 asm (" " "incq" " " "%0; sete %1"
       : "+m" (l->a.counter), "=qm" (c)
       : : "memory");
 return c != 0;
}
static inline int local_add_negative(long i, local_t *l)
{
 unsigned char c;

 asm (" " "addq" " " "%2,%0; sets %1"
       : "+m" (l->a.counter), "=qm" (c)
       : "ir" (i) : "memory");
 return c;
}
static inline long local_add_return(long i, local_t *l)
{
 long __i;






 __i = i;
 asm (" " "xaddq" " " "%0, %1;"
       : "+r" (i), "+m" (l->a.counter)
       : : "memory");
 return i + __i;
}

static inline long local_sub_return(long i, local_t *l)
{
 return local_add_return(-i, l);
}





struct mod_arch_specific {};
struct kernel_symbol
{
 unsigned long value;
 char *name;
};

struct modversion_info
{
 unsigned long crc;
 char name[(64 - sizeof(unsigned long))];
};

struct module;

struct module_attribute {
        struct attribute attr;
        ssize_t (*show)(struct module_attribute *, struct module *, char *);
        ssize_t (*store)(struct module_attribute *, struct module *,
    char *, size_t count);
 void (*setup)(struct module *, char *);
 int (*test)(struct module *);
 void (*free)(struct module *);
};

struct module_kobject
{
 struct kobject kobj;
 struct module *mod;
 struct kobject *drivers_dir;
};


extern int init_module(void);
extern void cleanup_module(void);


struct exception_table_entry;

 struct exception_table_entry *
search_extable( struct exception_table_entry *first,
        struct exception_table_entry *last,
        unsigned long value);
void sort_extable(struct exception_table_entry *start,
    struct exception_table_entry *finish);
void sort_main_extable(void);
 struct exception_table_entry *search_exception_tables(unsigned long add);

struct notifier_block;




void *__symbol_get( char *symbol);
void *__symbol_get_gpl( char *symbol);
struct module_ref
{
 local_t count;
} __attribute__((__aligned__((1 << (7)))));

enum module_state
{
 MODULE_STATE_LIVE,
 MODULE_STATE_COMING,
 MODULE_STATE_GOING,
};

struct module
{
 enum module_state state;


 struct list_head list;


 char name[(64 - sizeof(unsigned long))];


 struct module_kobject mkobj;
 struct module_param_attrs *param_attrs;
 struct module_attribute *modinfo_attrs;
 char *version;
 char *srcversion;
 struct kobject *holders_dir;


 struct kernel_symbol *syms;
 unsigned long *crcs;
 unsigned int num_syms;


 unsigned int num_gpl_syms;
 struct kernel_symbol *gpl_syms;
 unsigned long *gpl_crcs;



 struct kernel_symbol *unused_syms;
 unsigned long *unused_crcs;
 unsigned int num_unused_syms;


 unsigned int num_unused_gpl_syms;
 struct kernel_symbol *unused_gpl_syms;
 unsigned long *unused_gpl_crcs;



 struct kernel_symbol *gpl_future_syms;
 unsigned long *gpl_future_crcs;
 unsigned int num_gpl_future_syms;


 unsigned int num_exentries;
 struct exception_table_entry *extable;


 int (*init)(void);


 void *module_init;


 void *module_core;


 unsigned int init_size, core_size;


 unsigned int init_text_size, core_text_size;


 void *unwind_info;


 struct mod_arch_specific arch;

 unsigned int taints;



 unsigned num_bugs;
 struct list_head bug_list;
 struct bug_entry *bug_table;




 Elf64_Sym *symtab;
 unsigned int num_symtab;
 char *strtab;


 struct module_sect_attrs *sect_attrs;


 struct module_notes_attrs *notes_attrs;



 void *percpu;



 char *args;

 struct marker *markers;
 unsigned int num_markers;




 struct list_head modules_which_use_me;


 struct task_struct *waiter;


 void (*exit)(void);


 struct module_ref ref[64];


};







static inline int module_is_live(struct module *mod)
{
 return mod->state != MODULE_STATE_GOING;
}


struct module *module_text_address(unsigned long addr);
struct module *__module_text_address(unsigned long addr);
int is_module_address(unsigned long addr);



int module_get_kallsym(unsigned int symnum, unsigned long *value, char *type,
   char *name, char *module_name, int *exported);


unsigned long module_kallsyms_lookup_name( char *name);

extern void __module_put_and_exit(struct module *mod, long code)
 __attribute__((noreturn));



unsigned int module_refcount(struct module *mod);
void __symbol_put( char *symbol);

void symbol_put_addr(void *addr);



static inline void __module_get(struct module *module)
{
 if (module) {
  local_inc(&module->ref[({ do { } while (0); ({ typeof(_proxy_pda.cpunumber) ret__; switch (sizeof(_proxy_pda.cpunumber)) { case 2: asm("mov" "w %%gs:%c1,%0" : "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->cpunumber)), "m" (_proxy_pda.cpunumber)); break; case 4: asm("mov" "l %%gs:%c1,%0": "=r" (ret__): "i" (((size_t) &((struct x8664_pda *)0)->cpunumber)), "m" (_proxy_pda.cpunumber)); break; case 8: asm("mov" "q %%gs:%c1,%0": "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->cpunumber)), "m" (_proxy_pda.cpunumber)); break; default: __bad_pda_field(); } ret__; }); })].count);
  do { } while (0);
 }
}

static inline int try_module_get(struct module *module)
{
 int ret = 1;

 if (module) {
  unsigned int cpu = ({ do { } while (0); ({ typeof(_proxy_pda.cpunumber) ret__; switch (sizeof(_proxy_pda.cpunumber)) { case 2: asm("mov" "w %%gs:%c1,%0" : "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->cpunumber)), "m" (_proxy_pda.cpunumber)); break; case 4: asm("mov" "l %%gs:%c1,%0": "=r" (ret__): "i" (((size_t) &((struct x8664_pda *)0)->cpunumber)), "m" (_proxy_pda.cpunumber)); break; case 8: asm("mov" "q %%gs:%c1,%0": "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->cpunumber)), "m" (_proxy_pda.cpunumber)); break; default: __bad_pda_field(); } ret__; }); });
  if (__builtin_expect(!!(module_is_live(module)), 1))
   local_inc(&module->ref[cpu].count);
  else
   ret = 0;
  do { } while (0);
 }
 return ret;
}

extern void module_put(struct module *module);
 char *module_address_lookup(unsigned long addr,
       unsigned long *symbolsize,
       unsigned long *offset,
       char **modname,
       char *namebuf);
int lookup_module_symbol_name(unsigned long addr, char *symname);
int lookup_module_symbol_attrs(unsigned long addr, unsigned long *size, unsigned long *offset, char *modname, char *name);


 struct exception_table_entry *search_module_extables(unsigned long addr);

int register_module_notifier(struct notifier_block * nb);
int unregister_module_notifier(struct notifier_block * nb);

extern void print_modules(void);

extern void module_update_markers(void);
struct device_driver;

struct module;

extern struct kset *module_kset;
extern struct kobj_type module_ktype;
extern int module_sysfs_initialized;

int mod_sysfs_init(struct module *mod);
int mod_sysfs_setup(struct module *mod,
      struct kernel_param *kparam,
      unsigned int num_params);
int module_add_modinfo_attrs(struct module *mod);
void module_remove_modinfo_attrs(struct module *mod);





static inline void *ERR_PTR(long error)
{
 return (void *) error;
}

static inline long PTR_ERR( void *ptr)
{
 return (long) ptr;
}

static inline long IS_ERR( void *ptr)
{
 return __builtin_expect(!!(((unsigned long)ptr) >= (unsigned long)-4095), 0);
}
static inline void *ERR_CAST( void *ptr)
{

 return (void *) ptr;
}


struct ts_config;
struct ts_state
{
 unsigned int offset;
 char cb[40];
};
struct ts_ops
{
 char *name;
 struct ts_config * (*init)( void *, unsigned int, gfp_t, int);
 unsigned int (*find)(struct ts_config *,
     struct ts_state *);
 void (*destroy)(struct ts_config *);
 void * (*get_pattern)(struct ts_config *);
 unsigned int (*get_pattern_len)(struct ts_config *);
 struct module *owner;
 struct list_head list;
};
struct ts_config
{
 struct ts_ops *ops;
 int flags;
 unsigned int (*get_next_block)(unsigned int consumed,
        u8 **dst,
        struct ts_config *conf,
        struct ts_state *state);
 void (*finish)(struct ts_config *conf,
       struct ts_state *state);
};
static inline unsigned int textsearch_next(struct ts_config *conf,
        struct ts_state *state)
{
 unsigned int ret = conf->ops->find(conf, state);

 if (conf->finish)
  conf->finish(conf, state);

 return ret;
}
static inline unsigned int textsearch_find(struct ts_config *conf,
        struct ts_state *state)
{
 state->offset = 0;
 return textsearch_next(conf, state);
}





static inline void *textsearch_get_pattern(struct ts_config *conf)
{
 return conf->ops->get_pattern(conf);
}





static inline unsigned int textsearch_get_pattern_len(struct ts_config *conf)
{
 return conf->ops->get_pattern_len(conf);
}

extern int textsearch_register(struct ts_ops *);
extern int textsearch_unregister(struct ts_ops *);
extern struct ts_config *textsearch_prepare( char *, void *,
         unsigned int, gfp_t, int);
extern void textsearch_destroy(struct ts_config *conf);
extern unsigned int textsearch_find_continuous(struct ts_config *,
            struct ts_state *,
            void *, unsigned int);





static inline struct ts_config *alloc_ts_config(size_t payload,
      gfp_t gfp_mask)
{
 struct ts_config *conf;

 conf = kzalloc((((sizeof(*conf)) + 8 -1) & ~(8 -1)) + payload, gfp_mask);
 if (conf == 0)
  return ERR_PTR(-12);

 return conf;
}

static inline void *ts_config_priv(struct ts_config *conf)
{
 return ((u8 *) conf + (((sizeof(struct ts_config)) + 8 -1) & ~(8 -1)));
}
struct exception_table_entry {
 unsigned long insn, fixup;
};

extern int fixup_exception(struct pt_regs *regs);
extern int __get_user_1(void);
extern int __get_user_2(void);
extern int __get_user_4(void);
extern int __get_user_8(void);
extern int __get_user_bad(void);
extern void __put_user_bad(void);





extern void __put_user_1(void);
extern void __put_user_2(void);
extern void __put_user_4(void);
extern void __put_user_8(void);
struct __large_struct { unsigned long buf[100]; };
__attribute__((warn_unused_result)) unsigned long
copy_user_generic(void *to, void *from, unsigned len);

__attribute__((warn_unused_result)) unsigned long
copy_to_user(void *to, void *from, unsigned len);
__attribute__((warn_unused_result)) unsigned long
copy_from_user(void *to, void *from, unsigned len);
__attribute__((warn_unused_result)) unsigned long
copy_in_user(void *to, void *from, unsigned len);

static inline __attribute__((always_inline)) __attribute__((warn_unused_result))
int __copy_from_user(void *dst, void *src, unsigned size)
{
 int ret = 0;
 if (!__builtin_constant_p(size))
  return copy_user_generic(dst, ( void *)src, size);
 switch (size) {
 case 1:asm ("1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r" (ret), "=q"(*(u8 *)dst) : "m" ((*(struct __large_struct *)((u8 *)src))), "i" (1), "0" (ret))
                                ;
  return ret;
 case 2:asm ("1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r" (ret), "=r"(*(u16 *)dst) : "m" ((*(struct __large_struct *)((u16 *)src))), "i" (2), "0" (ret))
                                ;
  return ret;
 case 4:asm ("1:	mov""l"" %2,%""k""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""l"" %""k""1,%""k""1\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r" (ret), "=r"(*(u32 *)dst) : "m" ((*(struct __large_struct *)((u32 *)src))), "i" (4), "0" (ret))
                                ;
  return ret;
 case 8:asm ("1:	mov""q"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""q"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r" (ret), "=r"(*(u64 *)dst) : "m" ((*(struct __large_struct *)((u64 *)src))), "i" (8), "0" (ret))
                               ;
  return ret;
 case 10:
  asm ("1:	mov""q"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""q"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r" (ret), "=r"(*(u64 *)dst) : "m" ((*(struct __large_struct *)((u64 *)src))), "i" (16), "0" (ret))
                                 ;
  if (__builtin_expect(!!(ret), 0))
   return ret;
  asm ("1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r" (ret), "=r"(*(u16 *)(8 + (char *)dst)) : "m" ((*(struct __large_struct *)((u16 *)(8 + (char *)src)))), "i" (2), "0" (ret))

                                 ;
  return ret;
 case 16:
  asm ("1:	mov""q"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""q"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r" (ret), "=r"(*(u64 *)dst) : "m" ((*(struct __large_struct *)((u64 *)src))), "i" (16), "0" (ret))
                                 ;
  if (__builtin_expect(!!(ret), 0))
   return ret;
  asm ("1:	mov""q"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""q"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r" (ret), "=r"(*(u64 *)(8 + (char *)dst)) : "m" ((*(struct __large_struct *)((u64 *)(8 + (char *)src)))), "i" (8), "0" (ret))

                                ;
  return ret;
 default:
  return copy_user_generic(dst, ( void *)src, size);
 }
}

static inline __attribute__((always_inline)) __attribute__((warn_unused_result))
int __copy_to_user(void *dst, void *src, unsigned size)
{
 int ret = 0;
 if (!__builtin_constant_p(size))
  return copy_user_generic(( void *)dst, src, size);
 switch (size) {
 case 1:asm ("1:	mov""b"" %""b""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r"(ret) : "iq"(*(u8 *)src), "m" ((*(struct __large_struct *)((u8 *)dst))), "i" (1), "0" (ret))
                                ;
  return ret;
 case 2:asm ("1:	mov""w"" %""w""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r"(ret) : "ir"(*(u16 *)src), "m" ((*(struct __large_struct *)((u16 *)dst))), "i" (2), "0" (ret))
                                ;
  return ret;
 case 4:asm ("1:	mov""l"" %""k""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r"(ret) : "ir"(*(u32 *)src), "m" ((*(struct __large_struct *)((u32 *)dst))), "i" (4), "0" (ret))
                                ;
  return ret;
 case 8:asm ("1:	mov""q"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r"(ret) : "ir"(*(u64 *)src), "m" ((*(struct __large_struct *)((u64 *)dst))), "i" (8), "0" (ret))
                               ;
  return ret;
 case 10:
  asm ("1:	mov""q"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r"(ret) : "ir"(*(u64 *)src), "m" ((*(struct __large_struct *)((u64 *)dst))), "i" (10), "0" (ret))
                                 ;
  if (__builtin_expect(!!(ret), 0))
   return ret;
  asm("":::"memory");
  asm ("1:	mov""w"" %""w""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r"(ret) : "ir"(4[(u16 *)src]), "m" ((*(struct __large_struct *)(4 + (u16 *)dst))), "i" (2), "0" (ret))
                                 ;
  return ret;
 case 16:
  asm ("1:	mov""q"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r"(ret) : "ir"(*(u64 *)src), "m" ((*(struct __large_struct *)((u64 *)dst))), "i" (16), "0" (ret))
                                 ;
  if (__builtin_expect(!!(ret), 0))
   return ret;
  asm("":::"memory");
  asm ("1:	mov""q"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r"(ret) : "ir"(1[(u64 *)src]), "m" ((*(struct __large_struct *)(1 + (u64 *)dst))), "i" (8), "0" (ret))
                                ;
  return ret;
 default:
  return copy_user_generic(( void *)dst, src, size);
 }
}

static inline __attribute__((always_inline)) __attribute__((warn_unused_result))
int __copy_in_user(void *dst, void *src, unsigned size)
{
 int ret = 0;
 if (!__builtin_constant_p(size))
  return copy_user_generic(( void *)dst,
      ( void *)src, size);
 switch (size) {
 case 1: {
  u8 tmp;
  asm ("1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r" (ret), "=q"(tmp) : "m" ((*(struct __large_struct *)((u8 *)src))), "i" (1), "0" (ret))
                                 ;
  if (__builtin_expect(!!(!ret), 1))
   asm ("1:	mov""b"" %""b""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r"(ret) : "iq"(tmp), "m" ((*(struct __large_struct *)((u8 *)dst))), "i" (1), "0" (ret))
                                  ;
  return ret;
 }
 case 2: {
  u16 tmp;
  asm ("1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r" (ret), "=r"(tmp) : "m" ((*(struct __large_struct *)((u16 *)src))), "i" (2), "0" (ret))
                                 ;
  if (__builtin_expect(!!(!ret), 1))
   asm ("1:	mov""w"" %""w""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r"(ret) : "ir"(tmp), "m" ((*(struct __large_struct *)((u16 *)dst))), "i" (2), "0" (ret))
                                  ;
  return ret;
 }

 case 4: {
  u32 tmp;
  asm ("1:	mov""l"" %2,%""k""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""l"" %""k""1,%""k""1\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r" (ret), "=r"(tmp) : "m" ((*(struct __large_struct *)((u32 *)src))), "i" (4), "0" (ret))
                                 ;
  if (__builtin_expect(!!(!ret), 1))
   asm ("1:	mov""l"" %""k""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r"(ret) : "ir"(tmp), "m" ((*(struct __large_struct *)((u32 *)dst))), "i" (4), "0" (ret))
                                  ;
  return ret;
 }
 case 8: {
  u64 tmp;
  asm ("1:	mov""q"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""q"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r" (ret), "=r"(tmp) : "m" ((*(struct __large_struct *)((u64 *)src))), "i" (8), "0" (ret))
                                ;
  if (__builtin_expect(!!(!ret), 1))
   asm ("1:	mov""q"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .section __ex_table,\"a\"\n" " " ".balign 8" " " "\n" " " ".quad" " " "1b" "," "3b" "\n" " .previous\n" : "=r"(ret) : "ir"(tmp), "m" ((*(struct __large_struct *)((u64 *)dst))), "i" (8), "0" (ret))
                                 ;
  return ret;
 }
 default:
  return copy_user_generic(( void *)dst,
      ( void *)src, size);
 }
}

__attribute__((warn_unused_result)) long
strncpy_from_user(char *dst, char *src, long count);
__attribute__((warn_unused_result)) long
__strncpy_from_user(char *dst, char *src, long count);
__attribute__((warn_unused_result)) long strnlen_user( char *str, long n);
__attribute__((warn_unused_result)) long __strnlen_user( char *str, long n);
__attribute__((warn_unused_result)) long strlen_user( char *str);
__attribute__((warn_unused_result)) unsigned long clear_user(void *mem, unsigned long len);
__attribute__((warn_unused_result)) unsigned long __clear_user(void *mem, unsigned long len);

__attribute__((warn_unused_result)) long __copy_from_user_inatomic(void *dst, void *src,
         unsigned size);

static __attribute__((warn_unused_result)) inline __attribute__((always_inline)) int
__copy_to_user_inatomic(void *dst, void *src, unsigned size)
{
 return copy_user_generic(( void *)dst, src, size);
}

extern long __copy_user_nocache(void *dst, void *src,
    unsigned size, int zerorest);

static inline int __copy_from_user_nocache(void *dst, void *src,
        unsigned size)
{
 do { __might_sleep("../../libos/2.6.27/include/asm/uaccess_64.h", 188); _cond_resched(); } while (0);
 return __copy_user_nocache(dst, src, size, 1);
}

static inline int __copy_from_user_inatomic_nocache(void *dst,
          void *src,
          unsigned size)
{
 return __copy_user_nocache(dst, src, size, 0);
}

unsigned long
copy_user_handle_tail(char *to, char *from, unsigned len, unsigned zerorest);



static inline __sum16 csum_fold(__wsum sum)
{
 asm("  addl %1,%0\n"
     "  adcl $0xffff,%0"
     : "=r" (sum)
     : "r" (( u32)sum << 16),
       "0" (( u32)sum & 0xffff0000));
 return ( __sum16)(~( u32)sum >> 16);
}
static inline __sum16 ip_fast_csum( void *iph, unsigned int ihl)
{
 unsigned int sum;

 asm("  movl (%1), %0\n"
     "  subl $4, %2\n"
     "  jbe 2f\n"
     "  addl 4(%1), %0\n"
     "  adcl 8(%1), %0\n"
     "  adcl 12(%1), %0\n"
     "1: adcl 16(%1), %0\n"
     "  lea 4(%1), %1\n"
     "  decl %2\n"
     "  jne	1b\n"
     "  adcl $0, %0\n"
     "  movl %0, %2\n"
     "  shrl $16, %0\n"
     "  addw %w2, %w0\n"
     "  adcl $0, %0\n"
     "  notl %0\n"
     "2:"



     : "=r" (sum), "=r" (iph), "=r" (ihl)
     : "1" (iph), "2" (ihl)
     : "memory");
 return ( __sum16)sum;
}
static inline __wsum
csum_tcpudp_nofold(__be32 saddr, __be32 daddr, unsigned short len,
     unsigned short proto, __wsum sum)
{
 asm("  addl %1, %0\n"
     "  adcl %2, %0\n"
     "  adcl %3, %0\n"
     "  adcl $0, %0\n"
     : "=r" (sum)
     : "g" (daddr), "g" (saddr),
       "g" ((len + proto)<<8), "0" (sum));
 return sum;
}
static inline __sum16 csum_tcpudp_magic(__be32 saddr, __be32 daddr,
     unsigned short len,
     unsigned short proto, __wsum sum)
{
 return csum_fold(csum_tcpudp_nofold(saddr, daddr, len, proto, sum));
}
extern __wsum csum_partial( void *buff, int len, __wsum sum);






extern __wsum csum_partial_copy_generic( void *src, void *dst,
     int len, __wsum sum,
     int *src_err_ptr, int *dst_err_ptr);


extern __wsum csum_partial_copy_from_user( void *src, void *dst,
       int len, __wsum isum, int *errp);
extern __wsum csum_partial_copy_to_user( void *src, void *dst,
     int len, __wsum isum, int *errp);
extern __wsum csum_partial_copy_nocheck( void *src, void *dst,
     int len, __wsum sum);
extern __sum16 ip_compute_csum( void *buff, int len);
struct in6_addr;


extern __sum16
csum_ipv6_magic( struct in6_addr *saddr, struct in6_addr *daddr,
  __u32 len, unsigned short proto, __wsum sum);

static inline unsigned add32_with_carry(unsigned a, unsigned b)
{
 asm("addl %2,%0\n\t"
     "adcl $0,%0"
     : "=r" (a)
     : "0" (a), "r" (b));
 return a;
}
static inline __wsum csum_add(__wsum csum, __wsum addend)
{
 u32 res = ( u32)csum;
 res += ( u32)addend;
 return ( __wsum)(res + (res < ( u32)addend));
}

static inline __wsum csum_sub(__wsum csum, __wsum addend)
{
 return csum_add(csum, ~addend);
}

static inline __wsum
csum_block_add(__wsum csum, __wsum csum2, int offset)
{
 u32 sum = ( u32)csum2;
 if (offset&1)
  sum = ((sum&0xFF00FF)<<8)+((sum>>8)&0xFF00FF);
 return csum_add(csum, ( __wsum)sum);
}

static inline __wsum
csum_block_sub(__wsum csum, __wsum csum2, int offset)
{
 u32 sum = ( u32)csum2;
 if (offset&1)
  sum = ((sum&0xFF00FF)<<8)+((sum>>8)&0xFF00FF);
 return csum_sub(csum, ( __wsum)sum);
}

static inline __wsum csum_unfold(__sum16 n)
{
 return ( __wsum)n;
}



static inline void csum_replace4(__sum16 *sum, __be32 from, __be32 to)
{
 __be32 diff[] = { ~from, to };

 *sum = csum_fold(csum_partial((char *)diff, sizeof(diff), ~csum_unfold(*sum)));
}

static inline void csum_replace2(__sum16 *sum, __be16 from, __be16 to)
{
 csum_replace4(sum, ( __be32)from, ( __be32)to);
}

struct sk_buff;
extern void inet_proto_csum_replace4(__sum16 *sum, struct sk_buff *skb,
         __be32 from, __be32 to, int pseudohdr);

static inline void inet_proto_csum_replace2(__sum16 *sum, struct sk_buff *skb,
         __be16 from, __be16 to,
         int pseudohdr)
{
 inet_proto_csum_replace4(sum, skb, ( __be32)from,
     ( __be32)to, pseudohdr);
}

struct resource {
 resource_size_t start;
 resource_size_t end;
 char *name;
 unsigned long flags;
 struct resource *parent, *sibling, *child;
};

struct resource_list {
 struct resource_list *next;
 struct resource *res;
 struct pci_dev *dev;
};
extern struct resource ioport_resource;
extern struct resource iomem_resource;

extern int request_resource(struct resource *root, struct resource *_new);
extern int release_resource(struct resource *_new);
extern int insert_resource(struct resource *parent, struct resource *_new);
extern void insert_resource_expand_to_fit(struct resource *root, struct resource *_new);
extern int allocate_resource(struct resource *root, struct resource *_new,
        resource_size_t size, resource_size_t min,
        resource_size_t max, resource_size_t align,
        void (*alignf)(void *, struct resource *,
         resource_size_t, resource_size_t),
        void *alignf_data);
int adjust_resource(struct resource *res, resource_size_t start,
      resource_size_t size);
resource_size_t resource_alignment(struct resource *res);
static inline resource_size_t resource_size(struct resource *res)
{
 return res->end - res->start + 1;
}






extern struct resource * __request_region(struct resource *,
     resource_size_t start,
     resource_size_t n, char *name);






extern int __check_region(struct resource *, resource_size_t, resource_size_t);
extern void __release_region(struct resource *, resource_size_t,
    resource_size_t);

static inline int check_region(resource_size_t s,
      resource_size_t n)
{
 return __check_region(&ioport_resource, s, n);
}


struct device;





extern struct resource * __devm_request_region(struct device *dev,
    struct resource *parent, resource_size_t start,
    resource_size_t n, char *name);






extern void __devm_release_region(struct device *dev, struct resource *parent,
      resource_size_t start, resource_size_t n);

struct klist_node;
struct klist {
 spinlock_t k_lock;
 struct list_head k_list;
 void (*get)(struct klist_node *);
 void (*put)(struct klist_node *);
};
extern void klist_init(struct klist *k, void (*get)(struct klist_node *),
         void (*put)(struct klist_node *));

struct klist_node {
 struct klist *n_klist;
 struct list_head n_node;
 struct kref n_ref;
 struct completion n_removed;
};

extern void klist_add_tail(struct klist_node *n, struct klist *k);
extern void klist_add_head(struct klist_node *n, struct klist *k);
extern void klist_add_after(struct klist_node *n, struct klist_node *pos);
extern void klist_add_before(struct klist_node *n, struct klist_node *pos);

extern void klist_del(struct klist_node *n);
extern void klist_remove(struct klist_node *n);

extern int klist_node_attached(struct klist_node *n);


struct klist_iter {
 struct klist *i_klist;
 struct list_head *i_head;
 struct klist_node *i_cur;
};


extern void klist_iter_init(struct klist *k, struct klist_iter *i);
extern void klist_iter_init_node(struct klist *k, struct klist_iter *i,
     struct klist_node *n);
extern void klist_iter_exit(struct klist_iter *i);
extern struct klist_node *klist_next(struct klist_iter *i);






struct semaphore {
 spinlock_t lock;
 unsigned int count;
 struct list_head wait_list;
};
static inline void sema_init(struct semaphore *sem, int val)
{





}




extern void down(struct semaphore *sem);
extern int __attribute__((warn_unused_result)) down_interruptible(struct semaphore *sem);
extern int __attribute__((warn_unused_result)) down_killable(struct semaphore *sem);
extern int __attribute__((warn_unused_result)) down_trylock(struct semaphore *sem);
extern int __attribute__((warn_unused_result)) down_timeout(struct semaphore *sem, long jiffies);
extern void up(struct semaphore *sem);



struct dev_archdata {

 void *acpi_handle;


struct dma_mapping_ops *dma_ops;




};



struct device;
struct device_driver;
struct driver_private;
struct _class;
struct class_private;
struct bus_type;
struct bus_type_private;

struct bus_attribute {
 struct attribute attr;
 ssize_t (*show)(struct bus_type *bus, char *buf);
 ssize_t (*store)(struct bus_type *bus, char *buf, size_t count);
};




extern int __attribute__((warn_unused_result)) bus_create_file(struct bus_type *,
     struct bus_attribute *);
extern void bus_remove_file(struct bus_type *, struct bus_attribute *);

struct bus_type {
 char *name;
 struct bus_attribute *bus_attrs;
 struct device_attribute *dev_attrs;
 struct driver_attribute *drv_attrs;

 int (*match)(struct device *dev, struct device_driver *drv);
 int (*uevent)(struct device *dev, struct kobj_uevent_env *env);
 int (*probe)(struct device *dev);
 int (*remove)(struct device *dev);
 void (*shutdown)(struct device *dev);

 int (*suspend)(struct device *dev, pm_message_t state);
 int (*suspend_late)(struct device *dev, pm_message_t state);
 int (*resume_early)(struct device *dev);
 int (*resume)(struct device *dev);

 struct pm_ext_ops *pm;

 struct bus_type_private *p;
};

extern int __attribute__((warn_unused_result)) bus_register(struct bus_type *bus);
extern void bus_unregister(struct bus_type *bus);

extern int __attribute__((warn_unused_result)) bus_rescan_devices(struct bus_type *bus);



int bus_for_each_dev(struct bus_type *bus, struct device *start, void *data,
       int (*fn)(struct device *dev, void *data));
struct device *bus_find_device(struct bus_type *bus, struct device *start,
          void *data,
          int (*match)(struct device *dev, void *data));
struct device *bus_find_device_by_name(struct bus_type *bus,
           struct device *start,
           char *name);

int __attribute__((warn_unused_result)) bus_for_each_drv(struct bus_type *bus,
      struct device_driver *start, void *data,
      int (*fn)(struct device_driver *, void *));







struct notifier_block;

extern int bus_register_notifier(struct bus_type *bus,
     struct notifier_block *nb);
extern int bus_unregister_notifier(struct bus_type *bus,
       struct notifier_block *nb);
extern struct kset *bus_get_kset(struct bus_type *bus);
extern struct klist *bus_get_device_klist(struct bus_type *bus);

struct device_driver {
 char *name;
 struct bus_type *bus;

 struct module *owner;
 char *mod_name;

 int (*probe) (struct device *dev);
 int (*remove) (struct device *dev);
 void (*shutdown) (struct device *dev);
 int (*suspend) (struct device *dev, pm_message_t state);
 int (*resume) (struct device *dev);
 struct attribute_group **groups;

 struct pm_ops *pm;

 struct driver_private *p;
};


extern int __attribute__((warn_unused_result)) driver_register(struct device_driver *drv);
extern void driver_unregister(struct device_driver *drv);

extern struct device_driver *get_driver(struct device_driver *drv);
extern void put_driver(struct device_driver *drv);
extern struct device_driver *driver_find( char *name,
      struct bus_type *bus);
extern int driver_probe_done(void);



struct driver_attribute {
 struct attribute attr;
 ssize_t (*show)(struct device_driver *driver, char *buf);
 ssize_t (*store)(struct device_driver *driver, char *buf,
    size_t count);
};





extern int __attribute__((warn_unused_result)) driver_create_file(struct device_driver *driver,
        struct driver_attribute *attr);
extern void driver_remove_file(struct device_driver *driver,
          struct driver_attribute *attr);

extern int __attribute__((warn_unused_result)) driver_add_kobj(struct device_driver *drv,
     struct kobject *kobj,
     char *fmt, ...);

extern int __attribute__((warn_unused_result)) driver_for_each_device(struct device_driver *drv,
            struct device *start,
            void *data,
            int (*fn)(struct device *dev,
        void *));
struct device *driver_find_device(struct device_driver *drv,
      struct device *start, void *data,
      int (*match)(struct device *dev, void *data));




struct _class {
 char *name;
 struct module *owner;

 struct class_attribute *class_attrs;
 struct device_attribute *dev_attrs;
 struct kobject *dev_kobj;

 int (*dev_uevent)(struct device *dev, struct kobj_uevent_env *env);

 void (*class_release)(struct _class *_class);
 void (*dev_release)(struct device *dev);

 int (*suspend)(struct device *dev, pm_message_t state);
 int (*resume)(struct device *dev);

 struct pm_ops *pm;
 struct class_private *p;
};

extern struct kobject *sysfs_dev_block_kobj;
extern struct kobject *sysfs_dev_char_kobj;
extern int __attribute__((warn_unused_result)) __class_register(struct _class *_class,
      struct lock_class_key *key);
extern void class_unregister(struct _class *_class);
extern int class_for_each_device(struct _class *_class, struct device *start,
     void *data,
     int (*fn)(struct device *dev, void *data));
extern struct device *class_find_device(struct _class *_class,
     struct device *start, void *data,
     int (*match)(struct device *, void *));

struct class_attribute {
 struct attribute attr;
 ssize_t (*show)(struct _class *_class, char *buf);
 ssize_t (*store)(struct _class *_class, char *buf, size_t count);
};




extern int __attribute__((warn_unused_result)) class_create_file(struct _class *_class,
       struct class_attribute *attr);
extern void class_remove_file(struct _class *_class,
         struct class_attribute *attr);

struct class_interface {
 struct list_head node;
 struct _class *_class;

 int (*add_dev) (struct device *, struct class_interface *);
 void (*remove_dev) (struct device *, struct class_interface *);
};

extern int __attribute__((warn_unused_result)) class_interface_register(struct class_interface *);
extern void class_interface_unregister(struct class_interface *);

extern struct _class * __attribute__((warn_unused_result)) __class_create(struct module *owner,
        char *name,
        struct lock_class_key *key);
extern void class_destroy(struct _class *cls);
struct device_type {
 char *name;
 struct attribute_group **groups;
 int (*uevent)(struct device *dev, struct kobj_uevent_env *env);
 void (*release)(struct device *dev);

 int (*suspend)(struct device *dev, pm_message_t state);
 int (*resume)(struct device *dev);

 struct pm_ops *pm;
};


struct device_attribute {
 struct attribute attr;
 ssize_t (*show)(struct device *dev, struct device_attribute *attr,
   char *buf);
 ssize_t (*store)(struct device *dev, struct device_attribute *attr,
    char *buf, size_t count);
};




extern int __attribute__((warn_unused_result)) device_create_file(struct device *device,
        struct device_attribute *entry);
extern void device_remove_file(struct device *dev,
          struct device_attribute *attr);
extern int __attribute__((warn_unused_result)) device_create_bin_file(struct device *dev,
            struct bin_attribute *attr);
extern void device_remove_bin_file(struct device *dev,
       struct bin_attribute *attr);
extern int device_schedule_callback_owner(struct device *dev,
  void (*func)(struct device *dev), struct module *owner);






typedef void (*dr_release_t)(struct device *dev, void *res);
typedef int (*dr_match_t)(struct device *dev, void *res, void *match_data);


extern void *__devres_alloc(dr_release_t release, size_t size, gfp_t gfp,
        char *name);





extern void devres_free(void *res);
extern void devres_add(struct device *dev, void *res);
extern void *devres_find(struct device *dev, dr_release_t release,
    dr_match_t match, void *match_data);
extern void *devres_get(struct device *dev, void *new_res,
   dr_match_t match, void *match_data);
extern void *devres_remove(struct device *dev, dr_release_t release,
      dr_match_t match, void *match_data);
extern int devres_destroy(struct device *dev, dr_release_t release,
     dr_match_t match, void *match_data);


extern void * __attribute__((warn_unused_result)) devres_open_group(struct device *dev, void *id,
          gfp_t gfp);
extern void devres_close_group(struct device *dev, void *id);
extern void devres_remove_group(struct device *dev, void *id);
extern int devres_release_group(struct device *dev, void *id);


extern void *devm_kzalloc(struct device *dev, size_t size, gfp_t gfp);
extern void devm_kfree(struct device *dev, void *p);

struct device_dma_parameters {




 unsigned int max_segment_size;
 unsigned long segment_boundary_mask;
};

struct device {
 struct klist klist_children;
 struct klist_node knode_parent;
 struct klist_node knode_driver;
 struct klist_node knode_bus;
 struct device *parent;

 struct kobject kobj;
 char bus_id[20];
 char *init_name;
 struct device_type *type;
 unsigned uevent_suppress:1;

 struct semaphore sem;



 struct bus_type *bus;
 struct device_driver *driver;

 void *driver_data;
 void *platform_data;

 struct dev_pm_info power;


 int numa_node;

 u64 *dma_mask;
 u64 coherent_dma_mask;





 struct device_dma_parameters *dma_parms;

 struct list_head dma_pools;

 struct dma_coherent_mem *dma_mem;


 struct dev_archdata archdata;

 spinlock_t devres_lock;
 struct list_head devres_head;

 struct list_head node;
 struct _class *_class;
 dev_t devt;
 struct attribute_group **groups;

 void (*release)(struct device *dev);
};


static inline void device_init_wakeup(struct device *dev, int val)
{
 dev->power.can_wakeup = dev->power.should_wakeup = !!val;
}

static inline void device_set_wakeup_capable(struct device *dev, int val)
{
 dev->power.can_wakeup = !!val;
}

static inline int device_can_wakeup(struct device *dev)
{
 return dev->power.can_wakeup;
}

static inline void device_set_wakeup_enable(struct device *dev, int val)
{
 dev->power.should_wakeup = !!val;
}

static inline int device_may_wakeup(struct device *dev)
{
 return dev->power.can_wakeup && dev->power.should_wakeup;
}

static inline char *dev_name( struct device *dev)
{

 return dev->bus_id;
}

extern int dev_set_name(struct device *dev, char *name, ...)
   __attribute__((format(printf, 2, 3)));


static inline int dev_to_node(struct device *dev)
{
 return dev->numa_node;
}
static inline void set_dev_node(struct device *dev, int node)
{
 dev->numa_node = node;
}
static inline void *dev_get_drvdata(struct device *dev)
{
 return dev->driver_data;
}

static inline void dev_set_drvdata(struct device *dev, void *data)
{
 dev->driver_data = data;
}

static inline int device_is_registered(struct device *dev)
{
 return dev->kobj.state_in_sysfs;
}

void driver_init(void);




extern int __attribute__((warn_unused_result)) device_register(struct device *dev);
extern void device_unregister(struct device *dev);
extern void device_initialize(struct device *dev);
extern int __attribute__((warn_unused_result)) device_add(struct device *dev);
extern void device_del(struct device *dev);
extern int device_for_each_child(struct device *dev, void *data,
       int (*fn)(struct device *dev, void *data));
extern struct device *device_find_child(struct device *dev, void *data,
    int (*match)(struct device *dev, void *data));
extern int device_rename(struct device *dev, char *new_name);
extern int device_move(struct device *dev, struct device *new_parent);





extern int __attribute__((warn_unused_result)) device_bind_driver(struct device *dev);
extern void device_release_driver(struct device *dev);
extern int __attribute__((warn_unused_result)) device_attach(struct device *dev);
extern int __attribute__((warn_unused_result)) driver_attach(struct device_driver *drv);
extern int __attribute__((warn_unused_result)) device_reprobe(struct device *dev);




extern struct device *device_create_vargs(struct _class *cls,
       struct device *parent,
       dev_t devt,
       void *drvdata,
       char *fmt,
       va_list vargs);
extern struct device *device_create(struct _class *cls, struct device *parent,
        dev_t devt, void *drvdata,
        char *fmt, ...)
        __attribute__((format(printf, 5, 6)));

extern void device_destroy(struct _class *cls, dev_t devt);







extern int (*platform_notify)(struct device *dev);

extern int (*platform_notify_remove)(struct device *dev);






extern struct device *get_device(struct device *dev);
extern void put_device(struct device *dev);



extern void device_shutdown(void);


extern void sysdev_shutdown(void);


extern char *dev_driver_string( struct device *dev);




enum dma_data_direction {
 DMA_BIDIRECTIONAL = 0,
 DMA_TO_DEVICE = 1,
 DMA_FROM_DEVICE = 2,
 DMA_NONE = 3,
};
static inline int valid_dma_direction(int dma_direction)
{
 return ((dma_direction == DMA_BIDIRECTIONAL) ||
  (dma_direction == DMA_TO_DEVICE) ||
  (dma_direction == DMA_FROM_DEVICE));
}

static inline int is_device_dma_capable(struct device *dev)
{
 return dev->dma_mask != 0 && *dev->dma_mask != 0x0ULL;
}











struct scatterlist {



 unsigned long page_link;
 unsigned int offset;
 unsigned int length;
 dma_addr_t dma_address;
 unsigned int dma_length;
};





struct task_struct;

extern int debug_locks;
extern int debug_locks_silent;




extern int debug_locks_off(void);
struct task_struct;
static inline void debug_show_all_locks(void)
{
}

static inline void __debug_show_held_locks(struct task_struct *task)
{
}

static inline void debug_show_held_locks(struct task_struct *task)
{
}

static inline void
debug_check_no_locks_freed( void *from, unsigned long len)
{
}

static inline void
debug_check_no_locks_held(struct task_struct *task)
{
}


struct mempolicy;
struct anon_vma;
struct file_ra_state;
struct user_struct;
struct writeback_control;


extern unsigned long max_mapnr;


extern unsigned long num_physpages;
extern void * high_memory;
extern int page_cluster;


extern int sysctl_legacy_va_layout;




extern unsigned long mmap_min_addr;


extern unsigned long empty_zero_page[((1UL) << 12) / sizeof(unsigned long)];


extern spinlock_t pgd_lock;
extern struct list_head pgd_list;





static inline int pte_dirty(pte_t pte)
{
 return pte_flags(pte) & (((pteval_t)(1)) << 6);
}

static inline int pte_young(pte_t pte)
{
 return pte_flags(pte) & (((pteval_t)(1)) << 5);
}

static inline int pte_write(pte_t pte)
{
 return pte_flags(pte) & (((pteval_t)(1)) << 1);
}

static inline int pte_file(pte_t pte)
{
 return pte_flags(pte) & (((pteval_t)(1)) << 6);
}

static inline int pte_huge(pte_t pte)
{
 return pte_flags(pte) & (((pteval_t)(1)) << 7);
}

static inline int pte_global(pte_t pte)
{
 return pte_flags(pte) & (((pteval_t)(1)) << 8);
}

static inline int pte_exec(pte_t pte)
{
 return !(pte_flags(pte) & (((pteval_t)(1)) << 63));
}

static inline int pte_special(pte_t pte)
{
 return pte_val(pte) & (((pteval_t)(1)) << 9);
}

static inline int pmd_large(pmd_t pte)
{
 return (pmd_val(pte) & ((((pteval_t)(1)) << 7) | (((pteval_t)(1)) << 0))) ==
  ((((pteval_t)(1)) << 7) | (((pteval_t)(1)) << 0));
}

static inline pte_t pte_mkclean(pte_t pte)
{
 return __pte(pte_val(pte) & ~(((pteval_t)(1)) << 6));
}

static inline pte_t pte_mkold(pte_t pte)
{
 return __pte(pte_val(pte) & ~(((pteval_t)(1)) << 5));
}

static inline pte_t pte_wrprotect(pte_t pte)
{
 return __pte(pte_val(pte) & ~(((pteval_t)(1)) << 1));
}

static inline pte_t pte_mkexec(pte_t pte)
{
 return __pte(pte_val(pte) & ~(((pteval_t)(1)) << 63));
}

static inline pte_t pte_mkdirty(pte_t pte)
{
 return __pte(pte_val(pte) | (((pteval_t)(1)) << 6));
}

static inline pte_t pte_mkyoung(pte_t pte)
{
 return __pte(pte_val(pte) | (((pteval_t)(1)) << 5));
}

static inline pte_t pte_mkwrite(pte_t pte)
{
 return __pte(pte_val(pte) | (((pteval_t)(1)) << 1));
}

static inline pte_t pte_mkhuge(pte_t pte)
{
 return __pte(pte_val(pte) | (((pteval_t)(1)) << 7));
}

static inline pte_t pte_clrhuge(pte_t pte)
{
 return __pte(pte_val(pte) & ~(((pteval_t)(1)) << 7));
}

static inline pte_t pte_mkglobal(pte_t pte)
{
 return __pte(pte_val(pte) | (((pteval_t)(1)) << 8));
}

static inline pte_t pte_clrglobal(pte_t pte)
{
 return __pte(pte_val(pte) & ~(((pteval_t)(1)) << 8));
}

static inline pte_t pte_mkspecial(pte_t pte)
{
 return __pte(pte_val(pte) | (((pteval_t)(1)) << 9));
}

extern pteval_t __supported_pte_mask;

static inline pte_t pfn_pte(unsigned long page_nr, pgprot_t pgprot)
{
 return __pte((((phys_addr_t)page_nr << 12) |
        ((pgprot).pgprot)) & __supported_pte_mask);
}

static inline pmd_t pfn_pmd(unsigned long page_nr, pgprot_t pgprot)
{
 return __pmd((((phys_addr_t)page_nr << 12) |
        ((pgprot).pgprot)) & __supported_pte_mask);
}

static inline pte_t pte_modify(pte_t pte, pgprot_t newprot)
{
 pteval_t val = pte_val(pte);





 val &= (((pteval_t)(((signed long)(~(((1UL) << 12)-1))) & ((phys_addr_t)(1ULL << 46) - 1))) | (((pteval_t)(1)) << 4) | (((pteval_t)(1)) << 3) | (((pteval_t)(1)) << 9) | (((pteval_t)(1)) << 5) | (((pteval_t)(1)) << 6));
 val |= ((newprot).pgprot) & (~(((pteval_t)(((signed long)(~(((1UL) << 12)-1))) & ((phys_addr_t)(1ULL << 46) - 1))) | (((pteval_t)(1)) << 4) | (((pteval_t)(1)) << 3) | (((pteval_t)(1)) << 9) | (((pteval_t)(1)) << 5) | (((pteval_t)(1)) << 6))) & __supported_pte_mask;

 return __pte(val);
}



static inline pgprot_t pgprot_modify(pgprot_t oldprot, pgprot_t newprot)
{
 pgprotval_t preservebits = ((oldprot).pgprot) & (((pteval_t)(((signed long)(~(((1UL) << 12)-1))) & ((phys_addr_t)(1ULL << 46) - 1))) | (((pteval_t)(1)) << 4) | (((pteval_t)(1)) << 3) | (((pteval_t)(1)) << 9) | (((pteval_t)(1)) << 5) | (((pteval_t)(1)) << 6));
 pgprotval_t addbits = ((newprot).pgprot);
 return ((pgprot_t) { (preservebits | addbits) } );
}







struct file;
pgprot_t phys_mem_access_prot(struct file *file, unsigned long pfn,
                              unsigned long size, pgprot_t vma_prot);
int phys_mem_access_prot_allowed(struct file *file, unsigned long pfn,
                              unsigned long size, pgprot_t *vma_prot);



void set_pte_vaddr(unsigned long vaddr, pte_t pte);





static inline void native_pagetable_setup_start(pgd_t *base) {}
static inline void native_pagetable_setup_done(pgd_t *base) {}
extern pud_t level3_kernel_pgt[512];
extern pud_t level3_ident_pgt[512];
extern pmd_t level2_kernel_pgt[512];
extern pmd_t level2_fixmap_pgt[512];
extern pmd_t level2_ident_pgt[512];
extern pgd_t init_level4_pgt[];



extern void paging_init(void);
struct mm_struct;

void set_pte_vaddr_pud(pud_t *pud_page, unsigned long vaddr, pte_t new_pte);


static inline void native_pte_clear(struct mm_struct *mm, unsigned long addr,
        pte_t *ptep)
{
 *ptep = native_make_pte(0);
}

static inline void native_set_pte(pte_t *ptep, pte_t pte)
{
 *ptep = pte;
}

static inline void native_set_pte_atomic(pte_t *ptep, pte_t pte)
{
 native_set_pte(ptep, pte);
}

static inline pte_t native_ptep_get_and_clear(pte_t *xp)
{

 return native_make_pte(((__typeof__(*(&xp->pte)))__xchg((unsigned long)(0), (&xp->pte), sizeof(*(&xp->pte)))));







}

static inline void native_set_pmd(pmd_t *pmdp, pmd_t pmd)
{
 *pmdp = pmd;
}

static inline void native_pmd_clear(pmd_t *pmd)
{
 native_set_pmd(pmd, native_make_pmd(0));
}

static inline void native_set_pud(pud_t *pudp, pud_t pud)
{
 *pudp = pud;
}

static inline void native_pud_clear(pud_t *pud)
{
 native_set_pud(pud, native_make_pud(0));
}

static inline void native_set_pgd(pgd_t *pgdp, pgd_t pgd)
{
 *pgdp = pgd;
}

static inline void native_pgd_clear(pgd_t *pgd)
{
 native_set_pgd(pgd, native_make_pgd(0));
}
static inline int pgd_bad(pgd_t pgd)
{
 return (pgd_val(pgd) & ~(((pteval_t)(((signed long)(~(((1UL) << 12)-1))) & ((phys_addr_t)(1ULL << 46) - 1))) | (((pteval_t)(1)) << 2))) != ((((pteval_t)(1)) << 0) | (((pteval_t)(1)) << 1) | (((pteval_t)(1)) << 5) | (((pteval_t)(1)) << 6));
}

static inline int pud_bad(pud_t pud)
{
 return (pud_val(pud) & ~(((pteval_t)(((signed long)(~(((1UL) << 12)-1))) & ((phys_addr_t)(1ULL << 46) - 1))) | (((pteval_t)(1)) << 2))) != ((((pteval_t)(1)) << 0) | (((pteval_t)(1)) << 1) | (((pteval_t)(1)) << 5) | (((pteval_t)(1)) << 6));
}

static inline int pmd_bad(pmd_t pmd)
{
 return (pmd_val(pmd) & ~(((pteval_t)(((signed long)(~(((1UL) << 12)-1))) & ((phys_addr_t)(1ULL << 46) - 1))) | (((pteval_t)(1)) << 2))) != ((((pteval_t)(1)) << 0) | (((pteval_t)(1)) << 1) | (((pteval_t)(1)) << 5) | (((pteval_t)(1)) << 6));
}
static inline int pgd_large(pgd_t pgd) { return 0; }
static inline int pud_large(pud_t pte)
{
 return (pud_val(pte) & ((((pteval_t)(1)) << 7) | (((pteval_t)(1)) << 0))) ==
  ((((pteval_t)(1)) << 7) | (((pteval_t)(1)) << 0));
}
extern int direct_gbpages;
extern int kern_addr_valid(unsigned long addr);
extern void cleanup_highmap(void);
enum {
 PG_LEVEL_NONE,
 PG_LEVEL_4K,
 PG_LEVEL_2M,
 PG_LEVEL_1G,
 PG_LEVEL_NUM
};


extern void update_page_count(int level, unsigned long pages);
extern pte_t *lookup_address(unsigned long address, unsigned int *level);


static inline pte_t native_local_ptep_get_and_clear(pte_t *ptep)
{
 pte_t res = *ptep;


 native_pte_clear(0, 0, ptep);
 return res;
}

static inline void native_set_pte_at(struct mm_struct *mm, unsigned long addr,
         pte_t *ptep , pte_t pte)
{
 native_set_pte(ptep, pte);
}
struct vm_area_struct;


extern int ptep_set_access_flags(struct vm_area_struct *vma,
     unsigned long address, pte_t *ptep,
     pte_t entry, int dirty);


extern int ptep_test_and_clear_young(struct vm_area_struct *vma,
         unsigned long addr, pte_t *ptep);


extern int ptep_clear_flush_young(struct vm_area_struct *vma,
      unsigned long address, pte_t *ptep);


static inline pte_t ptep_get_and_clear(struct mm_struct *mm, unsigned long addr,
           pte_t *ptep)
{
 pte_t pte = native_ptep_get_and_clear(ptep);
 pte_update(mm, addr, ptep);
 return pte;
}


static inline pte_t ptep_get_and_clear_full(struct mm_struct *mm,
         unsigned long addr, pte_t *ptep,
         int full)
{
 pte_t pte;
 if (full) {




  pte = native_local_ptep_get_and_clear(ptep);
 } else {
  pte = ptep_get_and_clear(mm, addr, ptep);
 }
 return pte;
}


static inline void ptep_set_wrprotect(struct mm_struct *mm,
          unsigned long addr, pte_t *ptep)
{
 clear_bit(1, (unsigned long *)&ptep->pte);
 pte_update(mm, addr, ptep);
}
static inline void clone_pgd_range(pgd_t *dst, pgd_t *src, int count)
{
       memcpy(dst, src, count * sizeof(pgd_t));
}


void pgd_clear_bad(pgd_t *);
void pud_clear_bad(pud_t *);
void pmd_clear_bad(pmd_t *);

static inline int pgd_none_or_clear_bad(pgd_t *pgd)
{
 if ((!pgd_val(*pgd)))
  return 1;
 if (__builtin_expect(!!(pgd_bad(*pgd)), 0)) {
  pgd_clear_bad(pgd);
  return 1;
 }
 return 0;
}

static inline int pud_none_or_clear_bad(pud_t *pud)
{
 if ((!pud_val(*pud)))
  return 1;
 if (__builtin_expect(!!(pud_bad(*pud)), 0)) {
  pud_clear_bad(pud);
  return 1;
 }
 return 0;
}

static inline int pmd_none_or_clear_bad(pmd_t *pmd)
{
 if ((!pmd_val((*pmd))))
  return 1;
 if (__builtin_expect(!!(pmd_bad(*pmd)), 0)) {
  pmd_clear_bad(pmd);
  return 1;
 }
 return 0;
}

static inline pte_t __ptep_modify_prot_start(struct mm_struct *mm,
          unsigned long addr,
          pte_t *ptep)
{





 return ptep_get_and_clear(mm, addr, ptep);
}

static inline void __ptep_modify_prot_commit(struct mm_struct *mm,
          unsigned long addr,
          pte_t *ptep, pte_t pte)
{




 set_pte_at(mm, addr, ptep, pte);
}
extern struct kmem_cache *vm_area_cachep;






struct vm_list_struct {
 struct vm_list_struct *next;
 struct vm_area_struct *vma;
};
extern pgprot_t protection_map[16];
struct vm_fault {
 unsigned int flags;
 unsigned long pgoff;
 void *virtual_address;

 struct page *page;




};






struct vm_operations_struct {
 void (*open)(struct vm_area_struct * area);
 void (*close)(struct vm_area_struct * area);
 int (*fault)(struct vm_area_struct *vma, struct vm_fault *vmf);



 int (*page_mkwrite)(struct vm_area_struct *vma, struct vm_fault *vmf);




 int (*access)(struct vm_area_struct *vma, unsigned long addr,
        void *buf, int len, int write);
 int (*set_policy)(struct vm_area_struct *vma, struct mempolicy *_new);
 struct mempolicy *(*get_policy)(struct vm_area_struct *vma,
     unsigned long addr);
 int (*migrate)(struct vm_area_struct *vma, nodemask_t *from,
  nodemask_t *to, unsigned long flags);

};

struct mmu_gather;
struct inode;
enum pageflags {
 PG_locked,
 PG_error,
 PG_referenced,
 PG_uptodate,
 PG_dirty,
 PG_lru,
 PG_active,
 PG_slab,
 PG_owner_priv_1,
 PG_arch_1,
 PG_reserved,
 PG_private,
 PG_writeback,

 PG_head,
 PG_tail,



 PG_swapcache,
 PG_mappedtodisk,
 PG_reclaim,
 PG_buddy,



 __NR_PAGEFLAGS,


 PG_checked = PG_owner_priv_1,


 PG_pinned = PG_owner_priv_1,
 PG_savepinned = PG_dirty,


 PG_slob_page = PG_active,
 PG_slob_free = PG_private,


 PG_slub_frozen = PG_active,
 PG_slub_debug = PG_error,
};
struct page;

static inline int PageLocked(struct page *page) { return (__builtin_constant_p((PG_locked)) ? constant_test_bit((PG_locked), (&page->flags)) : variable_test_bit((PG_locked), (&page->flags))); }
static inline int PageError(struct page *page) { return (__builtin_constant_p((PG_error)) ? constant_test_bit((PG_error), (&page->flags)) : variable_test_bit((PG_error), (&page->flags))); } static inline void SetPageError(struct page *page) { set_bit(PG_error, &page->flags); } static inline void ClearPageError(struct page *page) { clear_bit(PG_error, &page->flags); }
static inline int PageReferenced(struct page *page) { return (__builtin_constant_p((PG_referenced)) ? constant_test_bit((PG_referenced), (&page->flags)) : variable_test_bit((PG_referenced), (&page->flags))); } static inline void SetPageReferenced(struct page *page) { set_bit(PG_referenced, &page->flags); } static inline void ClearPageReferenced(struct page *page) { clear_bit(PG_referenced, &page->flags); } static inline int TestClearPageReferenced(struct page *page) { return test_and_clear_bit(PG_referenced, &page->flags); }
static inline int PageDirty(struct page *page) { return (__builtin_constant_p((PG_dirty)) ? constant_test_bit((PG_dirty), (&page->flags)) : variable_test_bit((PG_dirty), (&page->flags))); } static inline void SetPageDirty(struct page *page) { set_bit(PG_dirty, &page->flags); } static inline void ClearPageDirty(struct page *page) { clear_bit(PG_dirty, &page->flags); } static inline int TestSetPageDirty(struct page *page) { return test_and_set_bit(PG_dirty, &page->flags); } static inline int TestClearPageDirty(struct page *page) { return test_and_clear_bit(PG_dirty, &page->flags); } static inline void __ClearPageDirty(struct page *page) { __clear_bit(PG_dirty, &page->flags); }
static inline int PageLRU(struct page *page) { return (__builtin_constant_p((PG_lru)) ? constant_test_bit((PG_lru), (&page->flags)) : variable_test_bit((PG_lru), (&page->flags))); } static inline void SetPageLRU(struct page *page) { set_bit(PG_lru, &page->flags); } static inline void ClearPageLRU(struct page *page) { clear_bit(PG_lru, &page->flags); } static inline void __ClearPageLRU(struct page *page) { __clear_bit(PG_lru, &page->flags); }
static inline int PageActive(struct page *page) { return (__builtin_constant_p((PG_active)) ? constant_test_bit((PG_active), (&page->flags)) : variable_test_bit((PG_active), (&page->flags))); } static inline void SetPageActive(struct page *page) { set_bit(PG_active, &page->flags); } static inline void ClearPageActive(struct page *page) { clear_bit(PG_active, &page->flags); } static inline void __ClearPageActive(struct page *page) { __clear_bit(PG_active, &page->flags); }
static inline int PageSlab(struct page *page) { return (__builtin_constant_p((PG_slab)) ? constant_test_bit((PG_slab), (&page->flags)) : variable_test_bit((PG_slab), (&page->flags))); } static inline void __SetPageSlab(struct page *page) { __set_bit(PG_slab, &page->flags); } static inline void __ClearPageSlab(struct page *page) { __clear_bit(PG_slab, &page->flags); }
static inline int PageChecked(struct page *page) { return (__builtin_constant_p((PG_checked)) ? constant_test_bit((PG_checked), (&page->flags)) : variable_test_bit((PG_checked), (&page->flags))); } static inline void SetPageChecked(struct page *page) { set_bit(PG_checked, &page->flags); } static inline void ClearPageChecked(struct page *page) { clear_bit(PG_checked, &page->flags); }
static inline int PagePinned(struct page *page) { return (__builtin_constant_p((PG_pinned)) ? constant_test_bit((PG_pinned), (&page->flags)) : variable_test_bit((PG_pinned), (&page->flags))); } static inline void SetPagePinned(struct page *page) { set_bit(PG_pinned, &page->flags); } static inline void ClearPagePinned(struct page *page) { clear_bit(PG_pinned, &page->flags); } static inline int TestSetPagePinned(struct page *page) { return test_and_set_bit(PG_pinned, &page->flags); } static inline int TestClearPagePinned(struct page *page) { return test_and_clear_bit(PG_pinned, &page->flags); }
static inline int PageSavePinned(struct page *page) { return (__builtin_constant_p((PG_savepinned)) ? constant_test_bit((PG_savepinned), (&page->flags)) : variable_test_bit((PG_savepinned), (&page->flags))); } static inline void SetPageSavePinned(struct page *page) { set_bit(PG_savepinned, &page->flags); } static inline void ClearPageSavePinned(struct page *page) { clear_bit(PG_savepinned, &page->flags); };
static inline int PageReserved(struct page *page) { return (__builtin_constant_p((PG_reserved)) ? constant_test_bit((PG_reserved), (&page->flags)) : variable_test_bit((PG_reserved), (&page->flags))); } static inline void SetPageReserved(struct page *page) { set_bit(PG_reserved, &page->flags); } static inline void ClearPageReserved(struct page *page) { clear_bit(PG_reserved, &page->flags); } static inline void __ClearPageReserved(struct page *page) { __clear_bit(PG_reserved, &page->flags); }
       

static inline int PagePrivate(struct page *page) { return (__builtin_constant_p((PG_private)) ? constant_test_bit((PG_private), (&page->flags)) : variable_test_bit((PG_private), (&page->flags))); } static inline void SetPagePrivate(struct page *page) { set_bit(PG_private, &page->flags); } static inline void ClearPagePrivate(struct page *page) { clear_bit(PG_private, &page->flags); } static inline void __ClearPagePrivate(struct page *page) { __clear_bit(PG_private, &page->flags); }
 static inline void __SetPagePrivate(struct page *page) { __set_bit(PG_private, &page->flags); }
       

static inline int PageSlobPage(struct page *page) { return (__builtin_constant_p((PG_slob_page)) ? constant_test_bit((PG_slob_page), (&page->flags)) : variable_test_bit((PG_slob_page), (&page->flags))); } static inline void __SetPageSlobPage(struct page *page) { __set_bit(PG_slob_page, &page->flags); } static inline void __ClearPageSlobPage(struct page *page) { __clear_bit(PG_slob_page, &page->flags); }
static inline int PageSlobFree(struct page *page) { return (__builtin_constant_p((PG_slob_free)) ? constant_test_bit((PG_slob_free), (&page->flags)) : variable_test_bit((PG_slob_free), (&page->flags))); } static inline void __SetPageSlobFree(struct page *page) { __set_bit(PG_slob_free, &page->flags); } static inline void __ClearPageSlobFree(struct page *page) { __clear_bit(PG_slob_free, &page->flags); }

static inline int PageSlubFrozen(struct page *page) { return (__builtin_constant_p((PG_slub_frozen)) ? constant_test_bit((PG_slub_frozen), (&page->flags)) : variable_test_bit((PG_slub_frozen), (&page->flags))); } static inline void __SetPageSlubFrozen(struct page *page) { __set_bit(PG_slub_frozen, &page->flags); } static inline void __ClearPageSlubFrozen(struct page *page) { __clear_bit(PG_slub_frozen, &page->flags); }
static inline int PageSlubDebug(struct page *page) { return (__builtin_constant_p((PG_slub_debug)) ? constant_test_bit((PG_slub_debug), (&page->flags)) : variable_test_bit((PG_slub_debug), (&page->flags))); } static inline void __SetPageSlubDebug(struct page *page) { __set_bit(PG_slub_debug, &page->flags); } static inline void __ClearPageSlubDebug(struct page *page) { __clear_bit(PG_slub_debug, &page->flags); }





static inline int PageWriteback(struct page *page) { return (__builtin_constant_p((PG_writeback)) ? constant_test_bit((PG_writeback), (&page->flags)) : variable_test_bit((PG_writeback), (&page->flags))); } static inline int TestSetPageWriteback(struct page *page) { return test_and_set_bit(PG_writeback, &page->flags); } static inline int TestClearPageWriteback(struct page *page) { return test_and_clear_bit(PG_writeback, &page->flags); }
static inline int PageBuddy(struct page *page) { return (__builtin_constant_p((PG_buddy)) ? constant_test_bit((PG_buddy), (&page->flags)) : variable_test_bit((PG_buddy), (&page->flags))); } static inline void __SetPageBuddy(struct page *page) { __set_bit(PG_buddy, &page->flags); } static inline void __ClearPageBuddy(struct page *page) { __clear_bit(PG_buddy, &page->flags); }
static inline int PageMappedToDisk(struct page *page) { return (__builtin_constant_p((PG_mappedtodisk)) ? constant_test_bit((PG_mappedtodisk), (&page->flags)) : variable_test_bit((PG_mappedtodisk), (&page->flags))); } static inline void SetPageMappedToDisk(struct page *page) { set_bit(PG_mappedtodisk, &page->flags); } static inline void ClearPageMappedToDisk(struct page *page) { clear_bit(PG_mappedtodisk, &page->flags); }


static inline int PageReclaim(struct page *page) { return (__builtin_constant_p((PG_reclaim)) ? constant_test_bit((PG_reclaim), (&page->flags)) : variable_test_bit((PG_reclaim), (&page->flags))); } static inline void SetPageReclaim(struct page *page) { set_bit(PG_reclaim, &page->flags); } static inline void ClearPageReclaim(struct page *page) { clear_bit(PG_reclaim, &page->flags); } static inline int TestClearPageReclaim(struct page *page) { return test_and_clear_bit(PG_reclaim, &page->flags); }
static inline int PageReadahead(struct page *page) { return (__builtin_constant_p((PG_reclaim)) ? constant_test_bit((PG_reclaim), (&page->flags)) : variable_test_bit((PG_reclaim), (&page->flags))); } static inline void SetPageReadahead(struct page *page) { set_bit(PG_reclaim, &page->flags); } static inline void ClearPageReadahead(struct page *page) { clear_bit(PG_reclaim, &page->flags); }
static inline int PageHighMem(struct page *page) { return 0; }



static inline int PageSwapCache(struct page *page) { return (__builtin_constant_p((PG_swapcache)) ? constant_test_bit((PG_swapcache), (&page->flags)) : variable_test_bit((PG_swapcache), (&page->flags))); } static inline void SetPageSwapCache(struct page *page) { set_bit(PG_swapcache, &page->flags); } static inline void ClearPageSwapCache(struct page *page) { clear_bit(PG_swapcache, &page->flags); }







static inline int PageUncached(struct page *page) { return 0; }


static inline int PageUptodate(struct page *page)
{
 int ret = (__builtin_constant_p((PG_uptodate)) ? constant_test_bit((PG_uptodate), (&(page)->flags)) : variable_test_bit((PG_uptodate), (&(page)->flags)));
 if (ret)
  ;

 return ret;
}

static inline void __SetPageUptodate(struct page *page)
{
 ;
 __set_bit(PG_uptodate, &(page)->flags);
}

static inline void SetPageUptodate(struct page *page)
{
 ;
 set_bit(PG_uptodate, &(page)->flags);

}

static inline void ClearPageUptodate(struct page *page) { clear_bit(PG_uptodate, &page->flags); }

extern void cancel_dirty_page(struct page *page, unsigned int account_size);

int test_clear_page_writeback(struct page *page);
int test_set_page_writeback(struct page *page);

static inline void set_page_writeback(struct page *page)
{
 test_set_page_writeback(page);
}
static inline int PageHead(struct page *page) { return (__builtin_constant_p((PG_head)) ? constant_test_bit((PG_head), (&page->flags)) : variable_test_bit((PG_head), (&page->flags))); } static inline void __SetPageHead(struct page *page) { __set_bit(PG_head, &page->flags); } static inline void __ClearPageHead(struct page *page) { __clear_bit(PG_head, &page->flags); }
static inline int PageTail(struct page *page) { return (__builtin_constant_p((PG_tail)) ? constant_test_bit((PG_tail), (&page->flags)) : variable_test_bit((PG_tail), (&page->flags))); } static inline void __SetPageTail(struct page *page) { __set_bit(PG_tail, &page->flags); } static inline void __ClearPageTail(struct page *page) { __clear_bit(PG_tail, &page->flags); }

static inline int PageCompound(struct page *page)
{
 return page->flags & ((1L << PG_head) | (1L << PG_tail));

}
static inline int put_page_testzero(struct page *page)
{
 do { } while(0);
 return 1;
}





static inline int get_page_unless_zero(struct page *page)
{
 return 1;
}


struct page *vmalloc_to_page( void *addr);
unsigned long vmalloc_to_pfn( void *addr);







static inline int is_vmalloc_addr( void *x)
{

 unsigned long addr = (unsigned long)x;

 return addr >= (0xffffc20000000000UL) && addr < (0xffffe1ffffffffffUL);



}

static inline struct page *compound_head(struct page *page)
{
 if (__builtin_expect(!!(PageTail(page)), 0))
  return page->first_page;
 return page;
}

static inline int page_count(struct page *page)
{
 return ((&compound_head(page)->_count)->counter);
}

static inline void get_page(struct page *page)
{
 page = compound_head(page);
 do { } while(0);
 ;
}

static inline struct page *virt_to_head_page( void *x)
{
 struct page *page = (((struct page *)(0xffffe20000000000UL)) + (__phys_addr((unsigned long)(x)) >> 12));
 return compound_head(page);
}





static inline void init_page_count(struct page *page)
{
 ;
}






void put_pages_list(struct list_head *pages);

void split_page(struct page *page, unsigned int order);






typedef void compound_page_dtor(struct page *);

static inline void set_compound_page_dtor(struct page *page,
      compound_page_dtor *dtor)
{
 page[1].lru.next = (void *)dtor;
}

static inline compound_page_dtor *get_compound_page_dtor(struct page *page)
{
 return (compound_page_dtor *)page[1].lru.next;
}

static inline int compound_order(struct page *page)
{
 if (!PageHead(page))
  return 0;
 return (unsigned long)page[1].lru.prev;
}

static inline void set_compound_order(struct page *page, unsigned long order)
{
 page[1].lru.prev = (void *)order;
}
static inline enum zone_type page_zonenum(struct page *page)
{
 return (page->flags >> (((((sizeof(unsigned long)*8) - 0) - 9) - 2) * (2 != 0))) & ((1UL << 2) - 1);
}
static inline int page_zone_id(struct page *page)
{
 return (page->flags >> ((((((sizeof(unsigned long)*8) - 0) - 9) < ((((sizeof(unsigned long)*8) - 0) - 9) - 2))? (((sizeof(unsigned long)*8) - 0) - 9) : ((((sizeof(unsigned long)*8) - 0) - 9) - 2)) * ((9 + 2) != 0))) & ((1UL << (9 + 2)) - 1);
}

static inline int zone_to_nid(struct zone *zone)
{

 return zone->node;



}




static inline int page_to_nid(struct page *page)
{
 return (page->flags >> ((((sizeof(unsigned long)*8) - 0) - 9) * (9 != 0))) & ((1UL << 9) - 1);
}


static inline struct zone *page_zone(struct page *page)
{
 return &(node_data[page_to_nid(page)])->node_zones[page_zonenum(page)];
}
static inline void set_page_zone(struct page *page, enum zone_type zone)
{
 page->flags &= ~(((1UL << 2) - 1) << (((((sizeof(unsigned long)*8) - 0) - 9) - 2) * (2 != 0)));
 page->flags |= (zone & ((1UL << 2) - 1)) << (((((sizeof(unsigned long)*8) - 0) - 9) - 2) * (2 != 0));
}

static inline void set_page_node(struct page *page, unsigned long node)
{
 page->flags &= ~(((1UL << 9) - 1) << ((((sizeof(unsigned long)*8) - 0) - 9) * (9 != 0)));
 page->flags |= (node & ((1UL << 9) - 1)) << ((((sizeof(unsigned long)*8) - 0) - 9) * (9 != 0));
}

static inline void set_page_section(struct page *page, unsigned long section)
{
 page->flags &= ~(((1UL << 0) - 1) << (((sizeof(unsigned long)*8) - 0) * (0 != 0)));
 page->flags |= (section & ((1UL << 0) - 1)) << (((sizeof(unsigned long)*8) - 0) * (0 != 0));
}

static inline void set_page_links(struct page *page, enum zone_type zone,
 unsigned long node, unsigned long pfn)
{
 set_page_zone(page, zone);
 set_page_node(page, node);
 set_page_section(page, ((pfn) >> (27 - 12)));
}





static inline unsigned long round_hint_to_min(unsigned long hint)
{
 hint &= (~(((1UL) << 12)-1));
 if (((void *)hint != 0) &&
     (hint < mmap_min_addr))
  return (((mmap_min_addr)+((typeof(mmap_min_addr))(((1UL) << 12))-1))&~((typeof(mmap_min_addr))(((1UL) << 12))-1));
 return hint;
}










enum vm_event_item { PGPGIN, PGPGOUT, PSWPIN, PSWPOUT,
  PGALLOC_DMA, PGALLOC_DMA32, PGALLOC_NORMAL , PGALLOC_MOVABLE,
  PGFREE, PGACTIVATE, PGDEACTIVATE,
  PGFAULT, PGMAJFAULT,
  PGREFILL_DMA, PGREFILL_DMA32, PGREFILL_NORMAL , PGREFILL_MOVABLE,
  PGSTEAL_DMA, PGSTEAL_DMA32, PGSTEAL_NORMAL , PGSTEAL_MOVABLE,
  PGSCAN_KSWAPD_DMA, PGSCAN_KSWAPD_DMA32, PGSCAN_KSWAPD_NORMAL , PGSCAN_KSWAPD_MOVABLE,
  PGSCAN_DIRECT_DMA, PGSCAN_DIRECT_DMA32, PGSCAN_DIRECT_NORMAL , PGSCAN_DIRECT_MOVABLE,
  PGINODESTEAL, SLABS_SCANNED, KSWAPD_STEAL, KSWAPD_INODESTEAL,
  PAGEOUTRUN, ALLOCSTALL, PGROTATED,

  HTLB_BUDDY_PGALLOC, HTLB_BUDDY_PGALLOC_FAIL,

  NR_VM_EVENT_ITEMS
};

extern struct seq_operations fragmentation_op;
extern struct seq_operations pagetypeinfo_op;
extern struct seq_operations zoneinfo_op;
extern struct seq_operations vmstat_op;
extern int sysctl_stat_interval;
struct vm_event_state {
 unsigned long event[NR_VM_EVENT_ITEMS];
};

extern __typeof__(struct vm_event_state) per_cpu__vm_event_states;

static inline void __count_vm_event(enum vm_event_item item)
{
 (*({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((&per_cpu__vm_event_states))); (typeof((&per_cpu__vm_event_states))) (__ptr + ((({ typeof(_proxy_pda.data_offset) ret__; switch (sizeof(_proxy_pda.data_offset)) { case 2: asm("mov" "w %%gs:%c1,%0" : "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->data_offset)), "m" (_proxy_pda.data_offset)); break; case 4: asm("mov" "l %%gs:%c1,%0": "=r" (ret__): "i" (((size_t) &((struct x8664_pda *)0)->data_offset)), "m" (_proxy_pda.data_offset)); break; case 8: asm("mov" "q %%gs:%c1,%0": "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->data_offset)), "m" (_proxy_pda.data_offset)); break; default: __bad_pda_field(); } ret__; })))); })).event[item]++;
}

static inline void count_vm_event(enum vm_event_item item)
{
 (*({ extern int simple_identifier_vm_event_states(void); do { } while (0); &(*({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((&per_cpu__vm_event_states))); (typeof((&per_cpu__vm_event_states))) (__ptr + ((({ typeof(_proxy_pda.data_offset) ret__; switch (sizeof(_proxy_pda.data_offset)) { case 2: asm("mov" "w %%gs:%c1,%0" : "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->data_offset)), "m" (_proxy_pda.data_offset)); break; case 4: asm("mov" "l %%gs:%c1,%0": "=r" (ret__): "i" (((size_t) &((struct x8664_pda *)0)->data_offset)), "m" (_proxy_pda.data_offset)); break; case 8: asm("mov" "q %%gs:%c1,%0": "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->data_offset)), "m" (_proxy_pda.data_offset)); break; default: __bad_pda_field(); } ret__; })))); })); })).event[item]++;
 do { } while (0);
}

static inline void __count_vm_events(enum vm_event_item item, long delta)
{
 (*({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((&per_cpu__vm_event_states))); (typeof((&per_cpu__vm_event_states))) (__ptr + ((({ typeof(_proxy_pda.data_offset) ret__; switch (sizeof(_proxy_pda.data_offset)) { case 2: asm("mov" "w %%gs:%c1,%0" : "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->data_offset)), "m" (_proxy_pda.data_offset)); break; case 4: asm("mov" "l %%gs:%c1,%0": "=r" (ret__): "i" (((size_t) &((struct x8664_pda *)0)->data_offset)), "m" (_proxy_pda.data_offset)); break; case 8: asm("mov" "q %%gs:%c1,%0": "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->data_offset)), "m" (_proxy_pda.data_offset)); break; default: __bad_pda_field(); } ret__; })))); })).event[item] += delta;
}

static inline void count_vm_events(enum vm_event_item item, long delta)
{
 (*({ extern int simple_identifier_vm_event_states(void); do { } while (0); &(*({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((&per_cpu__vm_event_states))); (typeof((&per_cpu__vm_event_states))) (__ptr + ((({ typeof(_proxy_pda.data_offset) ret__; switch (sizeof(_proxy_pda.data_offset)) { case 2: asm("mov" "w %%gs:%c1,%0" : "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->data_offset)), "m" (_proxy_pda.data_offset)); break; case 4: asm("mov" "l %%gs:%c1,%0": "=r" (ret__): "i" (((size_t) &((struct x8664_pda *)0)->data_offset)), "m" (_proxy_pda.data_offset)); break; case 8: asm("mov" "q %%gs:%c1,%0": "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->data_offset)), "m" (_proxy_pda.data_offset)); break; default: __bad_pda_field(); } ret__; })))); })); })).event[item] += delta;
 do { } while (0);
}

extern void all_vm_events(unsigned long *);

extern void vm_events_fold_cpu(int cpu);
extern atomic_long_t vm_stat[NR_VM_ZONE_STAT_ITEMS];

static inline void zone_page_state_add(long x, struct zone *zone,
     enum zone_stat_item item)
{
 atomic_long_add(x, &zone->vm_stat[item]);
 atomic_long_add(x, &vm_stat[item]);
}

static inline unsigned long global_page_state(enum zone_stat_item item)
{
 long x = atomic_long_read(&vm_stat[item]);

 if (x < 0)
  x = 0;

 return x;
}

static inline unsigned long zone_page_state(struct zone *zone,
     enum zone_stat_item item)
{
 long x = atomic_long_read(&zone->vm_stat[item]);

 if (x < 0)
  x = 0;

 return x;
}







static inline unsigned long node_page_state(int node,
     enum zone_stat_item item)
{
 struct zone *zones = (node_data[node])->node_zones;

 return

  zone_page_state(&zones[ZONE_DMA], item) +


  zone_page_state(&zones[ZONE_DMA32], item) +




  zone_page_state(&zones[ZONE_NORMAL], item) +
  zone_page_state(&zones[ZONE_MOVABLE], item);
}

extern void zone_statistics(struct zone *, struct zone *);
static inline void zap_zone_vm_stats(struct zone *zone)
{
 memset(zone->vm_stat, 0, sizeof(zone->vm_stat));
}

extern void inc_zone_state(struct zone *, enum zone_stat_item);


void __mod_zone_page_state(struct zone *, enum zone_stat_item item, int);
void __inc_zone_page_state(struct page *, enum zone_stat_item);
void __dec_zone_page_state(struct page *, enum zone_stat_item);

void mod_zone_page_state(struct zone *, enum zone_stat_item, int);
void inc_zone_page_state(struct page *, enum zone_stat_item);
void dec_zone_page_state(struct page *, enum zone_stat_item);

extern void inc_zone_state(struct zone *, enum zone_stat_item);
extern void __inc_zone_state(struct zone *, enum zone_stat_item);
extern void dec_zone_state(struct zone *, enum zone_stat_item);
extern void __dec_zone_state(struct zone *, enum zone_stat_item);

void refresh_cpu_vm_stats(int);

static inline __attribute__((always_inline)) void *lowmem_page_address(struct page *page)
{
 return ((void *)((unsigned long)(((page) - ((struct page *)(0xffffe20000000000UL))) << 12)+((unsigned long)(0xffff880000000000UL))));
}
extern struct address_space swapper_space;
static inline struct address_space *page_mapping(struct page *page)
{
 struct address_space *mapping = page->mapping;

 do { } while(0);

 if (__builtin_expect(!!(PageSwapCache(page)), 0))
  mapping = &swapper_space;
 else

 if (__builtin_expect(!!((unsigned long)mapping & 1), 0))
  mapping = 0;
 return mapping;
}

static inline int PageAnon(struct page *page)
{
 return ((unsigned long)page->mapping & 1) != 0;
}





static inline unsigned long page_index(struct page *page)
{
 if (__builtin_expect(!!(PageSwapCache(page)), 0))
  return ((page)->_private);
 return page->index;
}






static inline void reset_page_mapcount(struct page *page)
{
 ;
}

static inline int page_mapcount(struct page *page)
{
 return ((&(page)->_mapcount)->counter) + 1;
}




static inline int page_mapped(struct page *page)
{
 return ((&(page)->_mapcount)->counter) >= 0;
}
extern void show_free_areas(void);


int shmem_lock(struct file *file, int lock, struct user_struct *user);







struct file *shmem_file_setup(char *name, loff_t size, unsigned long flags);

int shmem_zero_setup(struct vm_area_struct *);
extern int can_do_mlock(void);
extern int user_shm_lock(size_t, struct user_struct *);
extern void user_shm_unlock(size_t, struct user_struct *);




struct zap_details {
 struct vm_area_struct *nonlinear_vma;
 struct address_space *check_mapping;
 unsigned long first_index;
 unsigned long last_index;
 spinlock_t *i_mmap_lock;
 unsigned long truncate_count;
};

struct page *vm_normal_page(struct vm_area_struct *vma, unsigned long addr,
  pte_t pte);

int zap_vma_ptes(struct vm_area_struct *vma, unsigned long address,
  unsigned long size);
unsigned long zap_page_range(struct vm_area_struct *vma, unsigned long address,
  unsigned long size, struct zap_details *);
unsigned long unmap_vmas(struct mmu_gather **tlb,
  struct vm_area_struct *start_vma, unsigned long start_addr,
  unsigned long end_addr, unsigned long *nr_accounted,
  struct zap_details *);
struct mm_walk {
 int (*pgd_entry)(pgd_t *, unsigned long, unsigned long, struct mm_walk *);
 int (*pud_entry)(pud_t *, unsigned long, unsigned long, struct mm_walk *);
 int (*pmd_entry)(pmd_t *, unsigned long, unsigned long, struct mm_walk *);
 int (*pte_entry)(pte_t *, unsigned long, unsigned long, struct mm_walk *);
 int (*pte_hole)(unsigned long, unsigned long, struct mm_walk *);
 struct mm_struct *mm;
 void *_private;
};

int walk_page_range(unsigned long addr, unsigned long end,
  struct mm_walk *walk);
void free_pgd_range(struct mmu_gather *tlb, unsigned long addr,
  unsigned long end, unsigned long floor, unsigned long ceiling);
int copy_page_range(struct mm_struct *dst, struct mm_struct *src,
   struct vm_area_struct *vma);
void unmap_mapping_range(struct address_space *mapping,
  loff_t holebegin, loff_t holelen, int even_cows);
int generic_access_phys(struct vm_area_struct *vma, unsigned long addr,
   void *buf, int len, int write);

static inline void unmap_shared_mapping_range(struct address_space *mapping,
  loff_t holebegin, loff_t holelen)
{
 unmap_mapping_range(mapping, holebegin, holelen, 0);
}

extern int vmtruncate(struct inode * inode, loff_t offset);
extern int vmtruncate_range(struct inode * inode, loff_t offset, loff_t end);


extern int handle_mm_fault(struct mm_struct *mm, struct vm_area_struct *vma,
   unsigned long address, int write_access);
extern int make_pages_present(unsigned long addr, unsigned long end);
extern int access_process_vm(struct task_struct *tsk, unsigned long addr, void *buf, int len, int write);

int get_user_pages(struct task_struct *tsk, struct mm_struct *mm, unsigned long start,
  int len, int write, int force, struct page **pages, struct vm_area_struct **vmas);

extern int try_to_release_page(struct page * page, gfp_t gfp_mask);
extern void do_invalidatepage(struct page *page, unsigned long offset);

int __set_page_dirty_nobuffers(struct page *page);
int __set_page_dirty_no_writeback(struct page *page);
int redirty_page_for_writepage(struct writeback_control *wbc,
    struct page *page);
int set_page_dirty(struct page *page);
int set_page_dirty_lock(struct page *page);
int clear_page_dirty_for_io(struct page *page);

extern unsigned long move_page_tables(struct vm_area_struct *vma,
  unsigned long old_addr, struct vm_area_struct *new_vma,
  unsigned long new_addr, unsigned long len);
extern unsigned long do_mremap(unsigned long addr,
          unsigned long old_len, unsigned long new_len,
          unsigned long flags, unsigned long new_addr);
extern int mprotect_fixup(struct vm_area_struct *vma,
     struct vm_area_struct **pprev, unsigned long start,
     unsigned long end, unsigned long newflags);
int get_user_pages_fast(unsigned long start, int nr_pages, int write,
   struct page **pages);
struct shrinker {
 int (*shrink)(int nr_to_scan, gfp_t gfp_mask);
 int seeks;


 struct list_head list;
 long nr;
};

extern void register_shrinker(struct shrinker *);
extern void unregister_shrinker(struct shrinker *);

int vma_wants_writenotify(struct vm_area_struct *vma);

extern pte_t *get_locked_pte(struct mm_struct *mm, unsigned long addr, spinlock_t **ptl);
int __pud_alloc(struct mm_struct *mm, pgd_t *pgd, unsigned long address);
int __pmd_alloc(struct mm_struct *mm, pud_t *pud, unsigned long address);


int __pte_alloc(struct mm_struct *mm, pmd_t *pmd, unsigned long address);
int __pte_alloc_kernel(pmd_t *pmd, unsigned long address);






static inline pud_t *pud_alloc(struct mm_struct *mm, pgd_t *pgd, unsigned long address)
{
 return (__builtin_expect(!!((!pgd_val(*pgd))), 0) && __pud_alloc(mm, pgd, address))?
  0: ((pud_t *)((unsigned long)((void *)((unsigned long)((unsigned long)pgd_val((*(pgd))) & ((pteval_t)(((signed long)(~(((1UL) << 12)-1))) & ((phys_addr_t)(1ULL << 46) - 1))))+((unsigned long)(0xffff880000000000UL))))) + ((((address)) >> 30) & (512 - 1)));
}

static inline pmd_t *pmd_alloc(struct mm_struct *mm, pud_t *pud, unsigned long address)
{
 return (__builtin_expect(!!((!pud_val(*pud))), 0) && __pmd_alloc(mm, pud, address))?
  0: ((pmd_t *)((unsigned long)((void *)((unsigned long)(pud_val((*(pud))) & (((signed long)(~(((1UL) << 12)-1))) & ((phys_addr_t)(1ULL << 46) - 1)))+((unsigned long)(0xffff880000000000UL))))) + (((address) >> 21) & (512 - 1)));
}
static inline void pgtable_page_ctor(struct page *page)
{
 do { ; } while (0);
 inc_zone_page_state(page, NR_PAGETABLE);
}

static inline void pgtable_page_dtor(struct page *page)
{
 ((page)->mapping = 0);
 dec_zone_page_state(page, NR_PAGETABLE);
}
extern void free_area_init(unsigned long * zones_size);
extern void free_area_init_node(int nid, unsigned long * zones_size,
  unsigned long zone_start_pfn, unsigned long *zholes_size);
extern void free_area_init_nodes(unsigned long *max_zone_pfn);
extern void add_active_range(unsigned int nid, unsigned long start_pfn,
     unsigned long end_pfn);
extern void remove_active_range(unsigned int nid, unsigned long start_pfn,
     unsigned long end_pfn);
extern void push_node_boundaries(unsigned int nid, unsigned long start_pfn,
     unsigned long end_pfn);
extern void remove_all_active_ranges(void);
extern unsigned long absent_pages_in_range(unsigned long start_pfn,
      unsigned long end_pfn);
extern void get_pfn_range_for_nid(unsigned int nid,
   unsigned long *start_pfn, unsigned long *end_pfn);
extern unsigned long find_min_pfn_with_active_regions(void);
extern void free_bootmem_with_active_regions(int nid,
      unsigned long max_low_pfn);
typedef int (*work_fn_t)(unsigned long, unsigned long, void *);
extern void work_with_active_regions(int nid, work_fn_t work_fn, void *data);
extern void sparse_memory_present_with_active_regions(int nid);
extern int __attribute__ ((__section__(".meminit.text"))) __attribute__((__cold__)) early_pfn_to_nid(unsigned long pfn);


extern int __attribute__ ((__section__(".meminit.text"))) __attribute__((__cold__)) __early_pfn_to_nid(unsigned long pfn);



extern void set_dma_reserve(unsigned long new_dma_reserve);
extern void memmap_init_zone(unsigned long, int, unsigned long,
    unsigned long, enum memmap_context);
extern void setup_per_zone_pages_min(void);
extern void mem_init(void);
extern void show_mem(void);
extern void si_meminfo(struct sysinfo * val);
extern void si_meminfo_node(struct sysinfo *val, int nid);
extern int after_bootmem;


extern void setup_per_cpu_pageset(void);





void vma_prio_tree_add(struct vm_area_struct *, struct vm_area_struct *old);
void vma_prio_tree_insert(struct vm_area_struct *, struct prio_tree_root *);
void vma_prio_tree_remove(struct vm_area_struct *, struct prio_tree_root *);
struct vm_area_struct *vma_prio_tree_next(struct vm_area_struct *vma,
 struct prio_tree_iter *iter);





static inline void vma_nonlinear_insert(struct vm_area_struct *vma,
     struct list_head *list)
{
 vma->shared.vm_set.parent = 0;
 list_add_tail(&vma->shared.vm_set.list, list);
}


extern int __vm_enough_memory(struct mm_struct *mm, long pages, int cap_sys_admin);
extern void vma_adjust(struct vm_area_struct *vma, unsigned long start,
 unsigned long end, unsigned long pgoff, struct vm_area_struct *insert);
extern struct vm_area_struct *vma_merge(struct mm_struct *,
 struct vm_area_struct *prev, unsigned long addr, unsigned long end,
 unsigned long vm_flags, struct anon_vma *, struct file *, unsigned long,
 struct mempolicy *);
extern struct anon_vma *find_mergeable_anon_vma(struct vm_area_struct *);
extern int split_vma(struct mm_struct *,
 struct vm_area_struct *, unsigned long addr, int new_below);
extern int insert_vm_struct(struct mm_struct *, struct vm_area_struct *);
extern void __vma_link_rb(struct mm_struct *, struct vm_area_struct *,
 struct rb_node **, struct rb_node *);
extern void unlink_file_vma(struct vm_area_struct *);
extern struct vm_area_struct *copy_vma(struct vm_area_struct **,
 unsigned long addr, unsigned long len, unsigned long pgoff);
extern void exit_mmap(struct mm_struct *);

extern int mm_take_all_locks(struct mm_struct *mm);
extern void mm_drop_all_locks(struct mm_struct *mm);



extern void added_exe_file_vma(struct mm_struct *mm);
extern void removed_exe_file_vma(struct mm_struct *mm);
extern int may_expand_vm(struct mm_struct *mm, unsigned long npages);
extern int install_special_mapping(struct mm_struct *mm,
       unsigned long addr, unsigned long len,
       unsigned long flags, struct page **pages);

extern unsigned long get_unmapped_area(struct file *, unsigned long, unsigned long, unsigned long, unsigned long);

extern unsigned long do_mmap_pgoff(struct file *file, unsigned long addr,
 unsigned long len, unsigned long prot,
 unsigned long flag, unsigned long pgoff);
extern unsigned long mmap_region(struct file *file, unsigned long addr,
 unsigned long len, unsigned long flags,
 unsigned int vm_flags, unsigned long pgoff,
 int accountable);

static inline unsigned long do_mmap(struct file *file, unsigned long addr,
 unsigned long len, unsigned long prot,
 unsigned long flag, unsigned long offset)
{
 unsigned long ret = -22;
 if ((offset + (((len)+((typeof(len))(((1UL) << 12))-1))&~((typeof(len))(((1UL) << 12))-1))) < offset)
  goto out;
 if (!(offset & ~(~(((1UL) << 12)-1))))
  ret = do_mmap_pgoff(file, addr, len, prot, flag, offset >> 12);
out:
 return ret;
}

extern int do_munmap(struct mm_struct *, unsigned long, size_t);

extern unsigned long do_brk(unsigned long, unsigned long);


extern unsigned long page_unuse(struct page *);
extern void truncate_inode_pages(struct address_space *, loff_t);
extern void truncate_inode_pages_range(struct address_space *,
           loff_t lstart, loff_t lend);


extern int filemap_fault(struct vm_area_struct *, struct vm_fault *);


int write_one_page(struct page *page, int wait);





int do_page_cache_readahead(struct address_space *mapping, struct file *filp,
   unsigned long offset, unsigned long nr_to_read);
int force_page_cache_readahead(struct address_space *mapping, struct file *filp,
   unsigned long offset, unsigned long nr_to_read);

void page_cache_sync_readahead(struct address_space *mapping,
          struct file_ra_state *ra,
          struct file *filp,
          unsigned long offset,
          unsigned long size);

void page_cache_async_readahead(struct address_space *mapping,
    struct file_ra_state *ra,
    struct file *filp,
    struct page *pg,
    unsigned long offset,
    unsigned long size);

unsigned long max_sane_readahead(unsigned long nr);


extern int expand_stack(struct vm_area_struct *vma, unsigned long address);



extern int expand_stack_downwards(struct vm_area_struct *vma,
      unsigned long address);


extern struct vm_area_struct * find_vma(struct mm_struct * mm, unsigned long addr);
extern struct vm_area_struct * find_vma_prev(struct mm_struct * mm, unsigned long addr,
          struct vm_area_struct **pprev);



static inline struct vm_area_struct * find_vma_intersection(struct mm_struct * mm, unsigned long start_addr, unsigned long end_addr)
{
 struct vm_area_struct * vma = find_vma(mm,start_addr);

 if (vma && end_addr <= vma->vm_start)
  vma = 0;
 return vma;
}

static inline unsigned long vma_pages(struct vm_area_struct *vma)
{
 return (vma->vm_end - vma->vm_start) >> 12;
}

pgprot_t vm_get_page_prot(unsigned long vm_flags);
struct vm_area_struct *find_extend_vma(struct mm_struct *, unsigned long addr);
int remap_pfn_range(struct vm_area_struct *, unsigned long addr,
   unsigned long pfn, unsigned long size, pgprot_t);
int vm_insert_page(struct vm_area_struct *, unsigned long addr, struct page *);
int vm_insert_pfn(struct vm_area_struct *vma, unsigned long addr,
   unsigned long pfn);
int vm_insert_mixed(struct vm_area_struct *vma, unsigned long addr,
   unsigned long pfn);

struct page *follow_page(struct vm_area_struct *, unsigned long address,
   unsigned int foll_flags);





typedef int (*pte_fn_t)(pte_t *pte, pgtable_t token, unsigned long addr,
   void *data);
extern int apply_to_page_range(struct mm_struct *mm, unsigned long address,
          unsigned long size, pte_fn_t fn, void *data);


void vm_stat_account(struct mm_struct *, unsigned long, struct file *, long);
static inline void
kernel_map_pages(struct page *page, int numpages, int enable) {}
static inline void enable_debug_pagealloc(void)
{
}

static inline bool kernel_page_present(struct page *page) { return _true; }



extern struct vm_area_struct *get_gate_vma(struct task_struct *tsk);

int in_gate_area_no_task(unsigned long addr);
int in_gate_area(struct task_struct *task, unsigned long addr);





int drop_caches_sysctl_handler(struct ctl_table *, int, struct file *,
     void *, size_t *, loff_t *);
unsigned long shrink_slab(unsigned long scanned, gfp_t gfp_mask,
   unsigned long lru_pages);




extern int randomize_va_space;


 char * arch_vma_name(struct vm_area_struct *vma);
void print_vma_addr(char *prefix, unsigned long rip);

struct page *sparse_mem_map_populate(unsigned long pnum, int nid);
pgd_t *vmemmap_pgd_populate(unsigned long addr, int node);
pud_t *vmemmap_pud_populate(pgd_t *pgd, unsigned long addr, int node);
pmd_t *vmemmap_pmd_populate(pud_t *pud, unsigned long addr, int node);
pte_t *vmemmap_pte_populate(pmd_t *pmd, unsigned long addr, int node);
void *vmemmap_alloc_block(unsigned long size, int node);
void vmemmap_verify(pte_t *, int, unsigned long, unsigned long);
int vmemmap_populate_basepages(struct page *start_page,
      unsigned long pages, int node);
int vmemmap_populate(struct page *start_page, unsigned long pages, int node);
void vmemmap_populate_print_last(void);

extern void early_ioremap_init(void);
extern void early_ioremap_clear(void);
extern void early_ioremap_reset(void);
extern void *early_ioremap(unsigned long offset, unsigned long size);
extern void early_iounmap(void *addr, unsigned long size);
extern void *fix_ioremap(unsigned idx, unsigned long phys);
static inline unsigned char readb( void *addr) { unsigned char ret; asm ("mov" "b" " %1,%0":"=q" (ret) :"m" (*( unsigned char *)addr) :"memory"); return ret; }
static inline unsigned short readw( void *addr) { unsigned short ret; asm ("mov" "w" " %1,%0":"=r" (ret) :"m" (*( unsigned short *)addr) :"memory"); return ret; }
static inline unsigned int readl( void *addr) { unsigned int ret; asm ("mov" "l" " %1,%0":"=r" (ret) :"m" (*( unsigned int *)addr) :"memory"); return ret; }

static inline unsigned char __readb( void *addr) { unsigned char ret; asm ("mov" "b" " %1,%0":"=q" (ret) :"m" (*( unsigned char *)addr) ); return ret; }
static inline unsigned short __readw( void *addr) { unsigned short ret; asm ("mov" "w" " %1,%0":"=r" (ret) :"m" (*( unsigned short *)addr) ); return ret; }
static inline unsigned int __readl( void *addr) { unsigned int ret; asm ("mov" "l" " %1,%0":"=r" (ret) :"m" (*( unsigned int *)addr) ); return ret; }

static inline void writeb(unsigned char val, void *addr) { asm ("mov" "b" " %0,%1": :"q" (val), "m" (*( unsigned char *)addr) :"memory"); }
static inline void writew(unsigned short val, void *addr) { asm ("mov" "w" " %0,%1": :"r" (val), "m" (*( unsigned short *)addr) :"memory"); }
static inline void writel(unsigned int val, void *addr) { asm ("mov" "l" " %0,%1": :"r" (val), "m" (*( unsigned int *)addr) :"memory"); }

static inline void __writeb(unsigned char val, void *addr) { asm ("mov" "b" " %0,%1": :"q" (val), "m" (*( unsigned char *)addr) ); }
static inline void __writew(unsigned short val, void *addr) { asm ("mov" "w" " %0,%1": :"r" (val), "m" (*( unsigned short *)addr) ); }
static inline void __writel(unsigned int val, void *addr) { asm ("mov" "l" " %0,%1": :"r" (val), "m" (*( unsigned int *)addr) ); }
static inline unsigned long readq( void *addr) { unsigned long ret; asm ("mov" "q" " %1,%0":"=r" (ret) :"m" (*( unsigned long *)addr) :"memory"); return ret; }
static inline unsigned long __readq( void *addr) { unsigned long ret; asm ("mov" "q" " %1,%0":"=r" (ret) :"m" (*( unsigned long *)addr) ); return ret; }
static inline void writeq(unsigned long val, void *addr) { asm ("mov" "q" " %0,%1": :"r" (val), "m" (*( unsigned long *)addr) :"memory"); }
static inline void __writeq(unsigned long val, void *addr) { asm ("mov" "q" " %0,%1": :"r" (val), "m" (*( unsigned long *)addr) ); }
extern void native_io_delay(void);

extern int io_delay_type;
extern void io_delay_init(void);
static inline unsigned char inb(unsigned short port) { unsigned char _v; asm ("in" "b" " %" "w" "1,%" "" "0" : "=a" (_v) : "Nd" (port)); return _v; } static inline unsigned char inb_p(unsigned short port) { unsigned char _v; asm ("in" "b" " %" "w" "1,%" "" "0" : "=a" (_v) : "Nd" (port)); slow_down_io(); return _v; }


static inline unsigned short inw(unsigned short port) { unsigned short _v; asm ("in" "w" " %" "w" "1,%" "" "0" : "=a" (_v) : "Nd" (port)); return _v; } static inline unsigned short inw_p(unsigned short port) { unsigned short _v; asm ("in" "w" " %" "w" "1,%" "" "0" : "=a" (_v) : "Nd" (port)); slow_down_io(); return _v; }


static inline unsigned int inl(unsigned short port) { unsigned int _v; asm ("in" "l" " %" "w" "1,%" "" "0" : "=a" (_v) : "Nd" (port)); return _v; } static inline unsigned int inl_p(unsigned short port) { unsigned int _v; asm ("in" "l" " %" "w" "1,%" "" "0" : "=a" (_v) : "Nd" (port)); slow_down_io(); return _v; }


static inline void outb(unsigned char value, unsigned short port) { asm ("out" "b" " %" "b" "0,%" "w" "1" : : "a" (value), "Nd" (port)); } static inline void outb_p(unsigned char value, unsigned short port) { asm ("out" "b" " %" "b" "0,%" "w" "1" : : "a" (value), "Nd" (port)); slow_down_io(); }
static inline void outw(unsigned short value, unsigned short port) { asm ("out" "w" " %" "w" "0,%" "w" "1" : : "a" (value), "Nd" (port)); } static inline void outw_p(unsigned short value, unsigned short port) { asm ("out" "w" " %" "w" "0,%" "w" "1" : : "a" (value), "Nd" (port)); slow_down_io(); }
static inline void outl(unsigned int value, unsigned short port) { asm ("out" "l" " %" "0,%" "w" "1" : : "a" (value), "Nd" (port)); } static inline void outl_p(unsigned int value, unsigned short port) { asm ("out" "l" " %" "0,%" "w" "1" : : "a" (value), "Nd" (port)); slow_down_io(); }

static inline void insb(unsigned short port, void *addr, unsigned long count) { asm ("rep ; ins" "b" : "=D" (addr), "=c" (count) : "d" (port), "0" (addr), "1" (count)); }
static inline void insw(unsigned short port, void *addr, unsigned long count) { asm ("rep ; ins" "w" : "=D" (addr), "=c" (count) : "d" (port), "0" (addr), "1" (count)); }
static inline void insl(unsigned short port, void *addr, unsigned long count) { asm ("rep ; ins" "l" : "=D" (addr), "=c" (count) : "d" (port), "0" (addr), "1" (count)); }

static inline void outsb(unsigned short port, void *addr, unsigned long count) { asm ("rep ; outs" "b" : "=S" (addr), "=c" (count) : "d" (port), "0" (addr), "1" (count)); }
static inline void outsw(unsigned short port, void *addr, unsigned long count) { asm ("rep ; outs" "w" : "=S" (addr), "=c" (count) : "d" (port), "0" (addr), "1" (count)); }
static inline void outsl(unsigned short port, void *addr, unsigned long count) { asm ("rep ; outs" "l" : "=S" (addr), "=c" (count) : "d" (port), "0" (addr), "1" (count)); }











struct vm_area_struct;
struct vm_struct {

 struct vm_struct *next;
 void *addr;
 unsigned long size;
 unsigned long flags;
 struct page **pages;
 unsigned int nr_pages;
 unsigned long phys_addr;
 void *caller;
};




extern void *vmalloc(unsigned long size);
extern void *vmalloc_user(unsigned long size);
extern void *vmalloc_node(unsigned long size, int node);
extern void *vmalloc_exec(unsigned long size);
extern void *vmalloc_32(unsigned long size);
extern void *vmalloc_32_user(unsigned long size);
extern void *__vmalloc(unsigned long size, gfp_t gfp_mask, pgprot_t prot);
extern void *__vmalloc_area(struct vm_struct *area, gfp_t gfp_mask,
    pgprot_t prot);
extern void vfree( void *addr);

extern void *vmap(struct page **pages, unsigned int count,
   unsigned long flags, pgprot_t prot);
extern void vunmap( void *addr);

extern int remap_vmalloc_range(struct vm_area_struct *vma, void *addr,
       unsigned long pgoff);
void vmalloc_sync_all(void);





static inline size_t get_vm_area_size( struct vm_struct *area)
{

 return area->size - ((1UL) << 12);
}

extern struct vm_struct *get_vm_area(unsigned long size, unsigned long flags);
extern struct vm_struct *get_vm_area_caller(unsigned long size,
     unsigned long flags, void *caller);
extern struct vm_struct *__get_vm_area(unsigned long size, unsigned long flags,
     unsigned long start, unsigned long end);
extern struct vm_struct *get_vm_area_node(unsigned long size,
       unsigned long flags, int node,
       gfp_t gfp_mask);
extern struct vm_struct *remove_vm_area( void *addr);

extern int map_vm_area(struct vm_struct *area, pgprot_t prot,
   struct page ***pages);
extern void unmap_kernel_range(unsigned long addr, unsigned long size);


extern struct vm_struct *alloc_vm_area(size_t size);
extern void free_vm_area(struct vm_struct *area);




extern rwlock_t vmlist_lock;
extern struct vm_struct *vmlist;

extern struct seq_operations vmalloc_op;






static inline unsigned long virt_to_phys( void *address)
{
 return __phys_addr((unsigned long)(address));
}

static inline void *phys_to_virt(unsigned long address)
{
 return ((void *)((unsigned long)(address)+((unsigned long)(0xffff880000000000UL))));
}







extern unsigned int ioread8(void *);
extern unsigned int ioread16(void *);
extern unsigned int ioread16be(void *);
extern unsigned int ioread32(void *);
extern unsigned int ioread32be(void *);

extern void iowrite8(u8, void *);
extern void iowrite16(u16, void *);
extern void iowrite16be(u16, void *);
extern void iowrite32(u32, void *);
extern void iowrite32be(u32, void *);
extern void ioread8_rep(void *port, void *buf, unsigned long count);
extern void ioread16_rep(void *port, void *buf, unsigned long count);
extern void ioread32_rep(void *port, void *buf, unsigned long count);

extern void iowrite8_rep(void *port, void *buf, unsigned long count);
extern void iowrite16_rep(void *port, void *buf, unsigned long count);
extern void iowrite32_rep(void *port, void *buf, unsigned long count);


extern void *ioport_map(unsigned long port, unsigned int nr);
extern void ioport_unmap(void *);






struct pci_dev;
extern void *pci_iomap(struct pci_dev *dev, int bar, unsigned long max);
extern void pci_iounmap(struct pci_dev *dev, void *);

extern void *early_ioremap(unsigned long addr, unsigned long size);
extern void early_iounmap(void *addr, unsigned long size);






extern void *ioremap_nocache(resource_size_t offset, unsigned long size);
extern void *ioremap_cache(resource_size_t offset, unsigned long size);
extern void *ioremap_prot(resource_size_t offset, unsigned long size,
    unsigned long prot_val);




static inline void *ioremap(resource_size_t offset, unsigned long size)
{
 return ioremap_nocache(offset, size);
}

extern void iounmap( void *addr);

extern void *fix_ioremap(unsigned idx, unsigned long phys);
void __memcpy_fromio(void *, unsigned long, unsigned);
void __memcpy_toio(unsigned long, void *, unsigned);

static inline void memcpy_fromio(void *to, void *from,
     unsigned len)
{
 __memcpy_fromio(to, (unsigned long)from, len);
}

static inline void memcpy_toio( void *to, void *from,
          unsigned len)
{
 __memcpy_toio((unsigned long)to, from, len);
}

void memset_io( void *a, int b, size_t c);
extern int iommu_bio_merge;


extern void *xlate_dev_mem_ptr(unsigned long phys);
extern void unxlate_dev_mem_ptr(unsigned long phys, void *addr);

extern int ioremap_change_attr(unsigned long vaddr, unsigned long size,
    unsigned long prot_val);
extern void *ioremap_wc(unsigned long offset, unsigned long size);






extern void early_ioremap_init(void);
extern void early_ioremap_clear(void);
extern void early_ioremap_reset(void);
extern void *early_ioremap(unsigned long offset, unsigned long size);
extern void early_iounmap(void *addr, unsigned long size);
extern void *fix_ioremap(unsigned idx, unsigned long phys);

struct sg_table {
 struct scatterlist *sgl;
 unsigned int nents;
 unsigned int orig_nents;
};
static inline void sg_assign_page(struct scatterlist *sg, struct page *page)
{
 unsigned long page_link = sg->page_link & 0x3;





 ;




 sg->page_link = page_link | (unsigned long) page;
}
static inline void sg_set_page(struct scatterlist *sg, struct page *page,
          unsigned int len, unsigned int offset)
{
 sg_assign_page(sg, page);
 sg->offset = offset;
 sg->length = len;
}

static inline struct page *sg_page(struct scatterlist *sg)
{




 return (struct page *)((sg)->page_link & ~0x3);
}
static inline void sg_set_buf(struct scatterlist *sg, void *buf,
         unsigned int buflen)
{
 sg_set_page(sg, (((struct page *)(0xffffe20000000000UL)) + (__phys_addr((unsigned long)(buf)) >> 12)), buflen, ((unsigned long)(buf) & ~(~(((1UL) << 12)-1))));
}
static inline void sg_chain(struct scatterlist *prv, unsigned int prv_nents,
       struct scatterlist *sgl)
{







 prv[prv_nents - 1].offset = 0;
 prv[prv_nents - 1].length = 0;





 prv[prv_nents - 1].page_link = ((unsigned long) sgl | 0x01) & ~0x02;
}
static inline void sg_mark_end(struct scatterlist *sg)
{






 sg->page_link |= 0x02;
 sg->page_link &= ~0x01;
}
static inline dma_addr_t sg_phys(struct scatterlist *sg)
{
 return ((dma_addr_t)((sg_page(sg)) - ((struct page *)(0xffffe20000000000UL))) << 12) + sg->offset;
}
static inline void *sg_virt(struct scatterlist *sg)
{
 return lowmem_page_address(sg_page(sg)) + sg->offset;
}

struct scatterlist *sg_next(struct scatterlist *);
struct scatterlist *sg_last(struct scatterlist *s, unsigned int);
void sg_init_table(struct scatterlist *, unsigned int);
void sg_init_one(struct scatterlist *, void *, unsigned int);

typedef struct scatterlist *(sg_alloc_fn)(unsigned int, gfp_t);
typedef void (sg_free_fn)(struct scatterlist *, unsigned int);

void __sg_free_table(struct sg_table *, unsigned int, sg_free_fn *);
void sg_free_table(struct sg_table *);
int __sg_alloc_table(struct sg_table *, unsigned int, unsigned int, gfp_t,
       sg_alloc_fn *);
int sg_alloc_table(struct sg_table *, unsigned int, gfp_t);

size_t sg_copy_from_buffer(struct scatterlist *sgl, unsigned int nents,
      void *buf, size_t buflen);
size_t sg_copy_to_buffer(struct scatterlist *sgl, unsigned int nents,
    void *buf, size_t buflen);
struct sg_mapping_iter {

 struct page *page;
 void *addr;
 size_t length;
 size_t consumed;


 struct scatterlist *__sg;
 unsigned int __nents;
 unsigned int __offset;
 unsigned int __flags;
};

void sg_miter_start(struct sg_mapping_iter *miter, struct scatterlist *sgl,
      unsigned int nents, unsigned int flags);
bool sg_miter_next(struct sg_mapping_iter *miter);
void sg_miter_stop(struct sg_mapping_iter *miter);







extern dma_addr_t swiotlb_map_single(struct device *hwdev, void *ptr,
         size_t size, int dir);
extern void *swiotlb_alloc_coherent(struct device *hwdev, size_t size,
        dma_addr_t *dma_handle, gfp_t flags);
extern void swiotlb_unmap_single(struct device *hwdev, dma_addr_t dev_addr,
     size_t size, int dir);
extern void swiotlb_sync_single_for_cpu(struct device *hwdev,
     dma_addr_t dev_addr,
     size_t size, int dir);
extern void swiotlb_sync_single_for_device(struct device *hwdev,
        dma_addr_t dev_addr,
        size_t size, int dir);
extern void swiotlb_sync_single_range_for_cpu(struct device *hwdev,
           dma_addr_t dev_addr,
           unsigned long offset,
           size_t size, int dir);
extern void swiotlb_sync_single_range_for_device(struct device *hwdev,
       dma_addr_t dev_addr,
       unsigned long offset,
       size_t size, int dir);
extern void swiotlb_sync_sg_for_cpu(struct device *hwdev,
        struct scatterlist *sg, int nelems,
        int dir);
extern void swiotlb_sync_sg_for_device(struct device *hwdev,
           struct scatterlist *sg, int nelems,
           int dir);
extern int swiotlb_map_sg(struct device *hwdev, struct scatterlist *sg,
     int nents, int direction);
extern void swiotlb_unmap_sg(struct device *hwdev, struct scatterlist *sg,
        int nents, int direction);
extern int swiotlb_dma_mapping_error(struct device *hwdev, dma_addr_t dma_addr);
extern void swiotlb_free_coherent(struct device *hwdev, size_t size,
      void *vaddr, dma_addr_t dma_handle);
extern int swiotlb_dma_supported(struct device *hwdev, u64 mask);
extern void swiotlb_init(void);

extern int swiotlb_force;


extern int swiotlb;
extern void pci_swiotlb_init(void);







static inline void dma_mark_clean(void *addr, size_t size) {}

extern dma_addr_t bad_dma_address;
extern int iommu_merge;
extern struct device fallback_dev;
extern int panic_on_overflow;
extern int force_iommu;

struct dma_mapping_ops {
 int (*mapping_error)(struct device *dev,
      dma_addr_t dma_addr);
 void* (*alloc_coherent)(struct device *dev, size_t size,
    dma_addr_t *dma_handle, gfp_t gfp);
 void (*free_coherent)(struct device *dev, size_t size,
    void *vaddr, dma_addr_t dma_handle);
 dma_addr_t (*map_single)(struct device *hwdev, phys_addr_t ptr,
    size_t size, int direction);

 dma_addr_t (*map_simple)(struct device *hwdev, phys_addr_t ptr,
    size_t size, int direction);
 void (*unmap_single)(struct device *dev, dma_addr_t addr,
    size_t size, int direction);
 void (*sync_single_for_cpu)(struct device *hwdev,
    dma_addr_t dma_handle, size_t size,
    int direction);
 void (*sync_single_for_device)(struct device *hwdev,
    dma_addr_t dma_handle, size_t size,
    int direction);
 void (*sync_single_range_for_cpu)(struct device *hwdev,
    dma_addr_t dma_handle, unsigned long offset,
    size_t size, int direction);
 void (*sync_single_range_for_device)(struct device *hwdev,
    dma_addr_t dma_handle, unsigned long offset,
    size_t size, int direction);
 void (*sync_sg_for_cpu)(struct device *hwdev,
    struct scatterlist *sg, int nelems,
    int direction);
 void (*sync_sg_for_device)(struct device *hwdev,
    struct scatterlist *sg, int nelems,
    int direction);
 int (*map_sg)(struct device *hwdev, struct scatterlist *sg,
    int nents, int direction);
 void (*unmap_sg)(struct device *hwdev,
    struct scatterlist *sg, int nents,
    int direction);
 int (*dma_supported)(struct device *hwdev, u64 mask);
 int is_phys;
};

extern struct dma_mapping_ops *dma_ops;

static inline struct dma_mapping_ops *get_dma_ops(struct device *dev)
{



 if (__builtin_expect(!!(!dev), 0) || !dev->archdata.dma_ops)
  return dma_ops;
 else
  return dev->archdata.dma_ops;

}


static inline int dma_mapping_error(struct device *dev, dma_addr_t dma_addr)
{



 struct dma_mapping_ops *ops = get_dma_ops(dev);
 if (ops->mapping_error)
  return ops->mapping_error(dev, dma_addr);

 return (dma_addr == bad_dma_address);

}




void *dma_alloc_coherent(struct device *dev, size_t size,
      dma_addr_t *dma_handle, gfp_t flag);

void dma_free_coherent(struct device *dev, size_t size,
    void *vaddr, dma_addr_t dma_handle);


extern int dma_supported(struct device *hwdev, u64 mask);
extern int dma_set_mask(struct device *dev, u64 mask);

static inline dma_addr_t
dma_map_single(struct device *hwdev, void *ptr, size_t size,
        int direction)
{
 struct dma_mapping_ops *ops = get_dma_ops(hwdev);

 ;
 return ops->map_single(hwdev, virt_to_phys(ptr), size, direction);
}

static inline void
dma_unmap_single(struct device *dev, dma_addr_t addr, size_t size,
   int direction)
{
 struct dma_mapping_ops *ops = get_dma_ops(dev);

 ;
 if (ops->unmap_single)
  ops->unmap_single(dev, addr, size, direction);
}

static inline int
dma_map_sg(struct device *hwdev, struct scatterlist *sg,
    int nents, int direction)
{
 struct dma_mapping_ops *ops = get_dma_ops(hwdev);

 ;
 return ops->map_sg(hwdev, sg, nents, direction);
}

static inline void
dma_unmap_sg(struct device *hwdev, struct scatterlist *sg, int nents,
      int direction)
{
 struct dma_mapping_ops *ops = get_dma_ops(hwdev);

 ;
 if (ops->unmap_sg)
  ops->unmap_sg(hwdev, sg, nents, direction);
}

static inline void
dma_sync_single_for_cpu(struct device *hwdev, dma_addr_t dma_handle,
   size_t size, int direction)
{
 struct dma_mapping_ops *ops = get_dma_ops(hwdev);

 ;
 if (ops->sync_single_for_cpu)
  ops->sync_single_for_cpu(hwdev, dma_handle, size, direction);
 ;
}

static inline void
dma_sync_single_for_device(struct device *hwdev, dma_addr_t dma_handle,
      size_t size, int direction)
{
 struct dma_mapping_ops *ops = get_dma_ops(hwdev);

 ;
 if (ops->sync_single_for_device)
  ops->sync_single_for_device(hwdev, dma_handle, size, direction);
 ;
}

static inline void
dma_sync_single_range_for_cpu(struct device *hwdev, dma_addr_t dma_handle,
         unsigned long offset, size_t size, int direction)
{
 struct dma_mapping_ops *ops = get_dma_ops(hwdev);

 ;
 if (ops->sync_single_range_for_cpu)
  ops->sync_single_range_for_cpu(hwdev, dma_handle, offset,
            size, direction);
 ;
}

static inline void
dma_sync_single_range_for_device(struct device *hwdev, dma_addr_t dma_handle,
     unsigned long offset, size_t size,
     int direction)
{
 struct dma_mapping_ops *ops = get_dma_ops(hwdev);

 ;
 if (ops->sync_single_range_for_device)
  ops->sync_single_range_for_device(hwdev, dma_handle,
        offset, size, direction);
 ;
}

static inline void
dma_sync_sg_for_cpu(struct device *hwdev, struct scatterlist *sg,
      int nelems, int direction)
{
 struct dma_mapping_ops *ops = get_dma_ops(hwdev);

 ;
 if (ops->sync_sg_for_cpu)
  ops->sync_sg_for_cpu(hwdev, sg, nelems, direction);
 ;
}

static inline void
dma_sync_sg_for_device(struct device *hwdev, struct scatterlist *sg,
         int nelems, int direction)
{
 struct dma_mapping_ops *ops = get_dma_ops(hwdev);

 ;
 if (ops->sync_sg_for_device)
  ops->sync_sg_for_device(hwdev, sg, nelems, direction);

 ;
}

static inline dma_addr_t dma_map_page(struct device *dev, struct page *page,
          size_t offset, size_t size,
          int direction)
{
 struct dma_mapping_ops *ops = get_dma_ops(dev);

 ;
 return ops->map_single(dev, ((dma_addr_t)((page) - ((struct page *)(0xffffe20000000000UL))) << 12) + offset,
          size, direction);
}

static inline void dma_unmap_page(struct device *dev, dma_addr_t addr,
      size_t size, int direction)
{
 dma_unmap_single(dev, addr, size, direction);
}

static inline void
dma_cache_sync(struct device *dev, void *vaddr, size_t size,
 enum dma_data_direction dir)
{
 ;
}

static inline int dma_get_cache_alignment(void)
{


 return boot_cpu_data.x86_clflush_size;
}



extern u64 dma_get_required_mask(struct device *dev);

static inline unsigned int dma_get_max_seg_size(struct device *dev)
{
 return dev->dma_parms ? dev->dma_parms->max_segment_size : 65536;
}

static inline unsigned int dma_set_max_seg_size(struct device *dev,
      unsigned int size)
{
 if (dev->dma_parms) {
  dev->dma_parms->max_segment_size = size;
  return 0;
 } else
  return -5;
}

static inline unsigned long dma_get_seg_boundary(struct device *dev)
{
 return dev->dma_parms ?
  dev->dma_parms->segment_boundary_mask : 0xffffffff;
}

static inline int dma_set_seg_boundary(struct device *dev, unsigned long mask)
{
 if (dev->dma_parms) {
  dev->dma_parms->segment_boundary_mask = mask;
  return 0;
 } else
  return -5;
}
static inline int
dma_declare_coherent_memory(struct device *dev, dma_addr_t bus_addr,
       dma_addr_t device_addr, size_t size, int flags)
{
 return 0;
}

static inline void
dma_release_declared_memory(struct device *dev)
{
}

static inline void *
dma_mark_declared_memory_occupied(struct device *dev,
      dma_addr_t device_addr, size_t size)
{
 return ERR_PTR(-16);
}





extern void *dmam_alloc_coherent(struct device *dev, size_t size,
     dma_addr_t *dma_handle, gfp_t gfp);
extern void dmam_free_coherent(struct device *dev, size_t size, void *vaddr,
          dma_addr_t dma_handle);
extern void *dmam_alloc_noncoherent(struct device *dev, size_t size,
        dma_addr_t *dma_handle, gfp_t gfp);
extern void dmam_free_noncoherent(struct device *dev, size_t size, void *vaddr,
      dma_addr_t dma_handle);






static inline int dmam_declare_coherent_memory(struct device *dev,
    dma_addr_t bus_addr, dma_addr_t device_addr,
    size_t size, gfp_t gfp)
{
 return 0;
}

static inline void dmam_release_declared_memory(struct device *dev)
{
}



struct dma_attrs;
enum dma_state {
 DMA_RESOURCE_SUSPEND,
 DMA_RESOURCE_RESUME,
 DMA_RESOURCE_AVAILABLE,
 DMA_RESOURCE_REMOVED,
};







enum dma_state_client {
 DMA_ACK,
 DMA_DUP,
 DMA_NAK,
};






typedef s32 dma_cookie_t;
enum dma_status {
 DMA_SUCCESS,
 DMA_IN_PROGRESS,
 DMA_ERROR,
};




enum dma_transaction_type {
 DMA_MEMCPY,
 DMA_XOR,
 DMA_PQ_XOR,
 DMA_DUAL_XOR,
 DMA_PQ_UPDATE,
 DMA_ZERO_SUM,
 DMA_PQ_ZERO_SUM,
 DMA_MEMSET,
 DMA_MEMCPY_CRC32C,
 DMA_INTERRUPT,
 DMA_SLAVE,
};
enum dma_slave_width {
 DMA_SLAVE_WIDTH_8BIT,
 DMA_SLAVE_WIDTH_16BIT,
 DMA_SLAVE_WIDTH_32BIT,
};
enum dma_ctrl_flags {
 DMA_PREP_INTERRUPT = (1 << 0),
 DMA_CTRL_ACK = (1 << 1),
 DMA_COMPL_SKIP_SRC_UNMAP = (1 << 2),
 DMA_COMPL_SKIP_DEST_UNMAP = (1 << 3),
};





typedef struct { unsigned long bits[((((DMA_SLAVE + 1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))]; } dma_cap_mask_t;
struct dma_slave {
 struct device *dev;
 struct device *dma_dev;
 dma_addr_t tx_reg;
 dma_addr_t rx_reg;
 enum dma_slave_width reg_width;
};
struct dma_chan_percpu {
 local_t refcount;

 unsigned long memcpy_count;
 unsigned long bytes_transferred;
};
struct dma_chan {
 struct dma_device *device;
 dma_cookie_t cookie;


 int chan_id;
 struct device dev;

 struct kref refcount;
 int slow_ref;
 struct rcu_head rcu;

 struct list_head device_node;
 struct dma_chan_percpu *local;
 int client_count;
};



void dma_chan_cleanup(struct kref *kref);

static inline void dma_chan_get(struct dma_chan *chan)
{
 if (__builtin_expect(!!(chan->slow_ref), 0))
  kref_get(&chan->refcount);
 else {
  local_inc(&(({ struct percpu_data *__p = (struct percpu_data *)~(unsigned long)((chan->local)); (__typeof__((chan->local)))__p->ptrs[((({ do { } while (0); ({ typeof(_proxy_pda.cpunumber) ret__; switch (sizeof(_proxy_pda.cpunumber)) { case 2: asm("mov" "w %%gs:%c1,%0" : "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->cpunumber)), "m" (_proxy_pda.cpunumber)); break; case 4: asm("mov" "l %%gs:%c1,%0": "=r" (ret__): "i" (((size_t) &((struct x8664_pda *)0)->cpunumber)), "m" (_proxy_pda.cpunumber)); break; case 8: asm("mov" "q %%gs:%c1,%0": "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->cpunumber)), "m" (_proxy_pda.cpunumber)); break; default: __bad_pda_field(); } ret__; }); })))]; })->refcount));
  do { } while (0);
 }
}

static inline void dma_chan_put(struct dma_chan *chan)
{
 if (__builtin_expect(!!(chan->slow_ref), 0))
  kref_put(&chan->refcount, dma_chan_cleanup);
 else {
  local_dec(&(({ struct percpu_data *__p = (struct percpu_data *)~(unsigned long)((chan->local)); (__typeof__((chan->local)))__p->ptrs[((({ do { } while (0); ({ typeof(_proxy_pda.cpunumber) ret__; switch (sizeof(_proxy_pda.cpunumber)) { case 2: asm("mov" "w %%gs:%c1,%0" : "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->cpunumber)), "m" (_proxy_pda.cpunumber)); break; case 4: asm("mov" "l %%gs:%c1,%0": "=r" (ret__): "i" (((size_t) &((struct x8664_pda *)0)->cpunumber)), "m" (_proxy_pda.cpunumber)); break; case 8: asm("mov" "q %%gs:%c1,%0": "=r" (ret__) : "i" (((size_t) &((struct x8664_pda *)0)->cpunumber)), "m" (_proxy_pda.cpunumber)); break; default: __bad_pda_field(); } ret__; }); })))]; })->refcount));
  do { } while (0);
 }
}
struct dma_client;
typedef enum dma_state_client (*dma_event_callback) (struct dma_client *client,
  struct dma_chan *chan, enum dma_state state);
struct dma_client {
 dma_event_callback event_callback;
 dma_cap_mask_t cap_mask;
 struct dma_slave *slave;
 struct list_head global_node;
};

typedef void (*dma_async_tx_callback)(void *dma_async_param);
struct dma_async_tx_descriptor {
 dma_cookie_t cookie;
 enum dma_ctrl_flags flags;
 dma_addr_t phys;
 struct list_head tx_list;
 struct dma_chan *chan;
 dma_cookie_t (*tx_submit)(struct dma_async_tx_descriptor *tx);
 dma_async_tx_callback callback;
 void *callback_param;
 struct dma_async_tx_descriptor *next;
 struct dma_async_tx_descriptor *parent;
 spinlock_t lock;
};
struct dma_device {

 unsigned int chancnt;
 struct list_head channels;
 struct list_head global_node;
 dma_cap_mask_t cap_mask;
 int max_xor;

 struct kref refcount;
 struct completion done;

 int dev_id;
 struct device *dev;

 int (*device_alloc_chan_resources)(struct dma_chan *chan,
   struct dma_client *client);
 void (*device_free_chan_resources)(struct dma_chan *chan);

 struct dma_async_tx_descriptor *(*device_prep_dma_memcpy)(
  struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
  size_t len, unsigned long flags);
 struct dma_async_tx_descriptor *(*device_prep_dma_xor)(
  struct dma_chan *chan, dma_addr_t dest, dma_addr_t *src,
  unsigned int src_cnt, size_t len, unsigned long flags);
 struct dma_async_tx_descriptor *(*device_prep_dma_zero_sum)(
  struct dma_chan *chan, dma_addr_t *src, unsigned int src_cnt,
  size_t len, u32 *result, unsigned long flags);
 struct dma_async_tx_descriptor *(*device_prep_dma_memset)(
  struct dma_chan *chan, dma_addr_t dest, int value, size_t len,
  unsigned long flags);
 struct dma_async_tx_descriptor *(*device_prep_dma_interrupt)(
  struct dma_chan *chan, unsigned long flags);

 struct dma_async_tx_descriptor *(*device_prep_slave_sg)(
  struct dma_chan *chan, struct scatterlist *sgl,
  unsigned int sg_len, enum dma_data_direction direction,
  unsigned long flags);
 void (*device_terminate_all)(struct dma_chan *chan);

 enum dma_status (*device_is_tx_complete)(struct dma_chan *chan,
   dma_cookie_t cookie, dma_cookie_t *last,
   dma_cookie_t *used);
 void (*device_issue_pending)(struct dma_chan *chan);
};



void dma_async_client_register(struct dma_client *client);
void dma_async_client_unregister(struct dma_client *client);
void dma_async_client_chan_request(struct dma_client *client);
dma_cookie_t dma_async_memcpy_buf_to_buf(struct dma_chan *chan,
 void *dest, void *src, size_t len);
dma_cookie_t dma_async_memcpy_buf_to_pg(struct dma_chan *chan,
 struct page *page, unsigned int offset, void *kdata, size_t len);
dma_cookie_t dma_async_memcpy_pg_to_pg(struct dma_chan *chan,
 struct page *dest_pg, unsigned int dest_off, struct page *src_pg,
 unsigned int src_off, size_t len);
void dma_async_tx_descriptor_init(struct dma_async_tx_descriptor *tx,
 struct dma_chan *chan);

static inline void async_tx_ack(struct dma_async_tx_descriptor *tx)
{
 tx->flags |= DMA_CTRL_ACK;
}

static inline bool async_tx_test_ack(struct dma_async_tx_descriptor *tx)
{
 return (tx->flags & DMA_CTRL_ACK) == DMA_CTRL_ACK;
}


static inline int __first_dma_cap( dma_cap_mask_t *srcp)
{
 return ({ int __min1 = ((DMA_SLAVE + 1)); int __min2 = (find_first_bit(srcp->bits, (DMA_SLAVE + 1))); __min1 < __min2 ? __min1: __min2; })
                                              ;
}


static inline int __next_dma_cap(int n, dma_cap_mask_t *srcp)
{
 return ({ int __min1 = ((DMA_SLAVE + 1)); int __min2 = (find_next_bit(srcp->bits, (DMA_SLAVE + 1), n+1)); __min1 < __min2 ? __min1: __min2; })
                                                  ;
}


static inline void
__dma_cap_set(enum dma_transaction_type tx_type, dma_cap_mask_t *dstp)
{
 set_bit(tx_type, dstp->bits);
}


static inline int
__dma_has_cap(enum dma_transaction_type tx_type, dma_cap_mask_t *srcp)
{
 return (__builtin_constant_p((tx_type)) ? constant_test_bit((tx_type), (srcp->bits)) : variable_test_bit((tx_type), (srcp->bits)));
}
static inline void dma_async_issue_pending(struct dma_chan *chan)
{
 chan->device->device_issue_pending(chan);
}
static inline enum dma_status dma_async_is_tx_complete(struct dma_chan *chan,
 dma_cookie_t cookie, dma_cookie_t *last, dma_cookie_t *used)
{
 return chan->device->device_is_tx_complete(chan, cookie, last, used);
}
static inline enum dma_status dma_async_is_complete(dma_cookie_t cookie,
   dma_cookie_t last_complete, dma_cookie_t last_used)
{
 if (last_complete <= last_used) {
  if ((cookie <= last_complete) || (cookie > last_used))
   return DMA_SUCCESS;
 } else {
  if ((cookie <= last_complete) && (cookie > last_used))
   return DMA_SUCCESS;
 }
 return DMA_IN_PROGRESS;
}

enum dma_status dma_sync_wait(struct dma_chan *chan, dma_cookie_t cookie);



int dma_async_device_register(struct dma_device *device);
void dma_async_device_unregister(struct dma_device *device);



struct dma_page_list {
 char *base_address;
 int nr_pages;
 struct page **pages;
};

struct dma_pinned_list {
 int nr_iovecs;
 struct dma_page_list page_list[0];
};

struct dma_pinned_list *dma_pin_iovec_pages(struct iovec *iov, size_t len);
void dma_unpin_iovec_pages(struct dma_pinned_list* pinned_list);

dma_cookie_t dma_memcpy_to_iovec(struct dma_chan *chan, struct iovec *iov,
 struct dma_pinned_list *pinned_list, unsigned char *kdata, size_t len);
dma_cookie_t dma_memcpy_pg_to_iovec(struct dma_chan *chan, struct iovec *iov,
 struct dma_pinned_list *pinned_list, struct page *page,
 unsigned int offset, size_t len);
struct net_device;
struct scatterlist;
struct pipe_inode_info;


struct nf_conntrack {
 atomic_t use;
};



struct nf_bridge_info {
 atomic_t use;
 struct net_device *physindev;
 struct net_device *physoutdev;
 unsigned int mask;
 unsigned long data[32 / sizeof(unsigned long)];
};


struct sk_buff_head {

 struct sk_buff *next;
 struct sk_buff *prev;

 __u32 qlen;
 spinlock_t lock;
};

struct sk_buff;




typedef struct skb_frag_struct skb_frag_t;

struct skb_frag_struct {
 struct page *page;
 __u32 page_offset;
 __u32 size;
};




struct skb_shared_info {
 atomic_t dataref;
 unsigned short nr_frags;
 unsigned short gso_size;

 unsigned short gso_segs;
 unsigned short gso_type;
 __be32 ip6_frag_id;
 struct sk_buff *frag_list;
 skb_frag_t frags[(65536/((1UL) << 12) + 2)];
};
enum {
 SKB_FCLONE_UNAVAILABLE,
 SKB_FCLONE_ORIG,
 SKB_FCLONE_CLONE,
};

enum {
 SKB_GSO_TCPV4 = 1 << 0,
 SKB_GSO_UDP = 1 << 1,


 SKB_GSO_DODGY = 1 << 2,


 SKB_GSO_TCP_ECN = 1 << 3,

 SKB_GSO_TCPV6 = 1 << 4,
};






typedef unsigned int sk_buff_data_t;
struct sk_buff {

 struct sk_buff *next;
 struct sk_buff *prev;

 struct sock *sk;
 ktime_t tstamp;
 struct net_device *dev;

 union {
  struct dst_entry *dst;
  struct rtable *rtable;
 };
 struct sec_path *sp;







 char cb[48];

 unsigned int len,
    data_len;
 __u16 mac_len,
    hdr_len;
 union {
  __wsum csum;
  struct {
   __u16 csum_start;
   __u16 csum_offset;
  };
 };
 __u32 priority;
 __u8 local_df:1,
    cloned:1,
    ip_summed:2,
    nohdr:1,
    nfctinfo:3;
 __u8 pkt_type:3,
    fclone:2,
    ipvs_property:1,
    peeked:1,
    nf_trace:1;
 __be16 protocol;

 void (*destructor)(struct sk_buff *skb);

 struct nf_conntrack *nfct;
 struct sk_buff *nfct_reasm;


 struct nf_bridge_info *nf_bridge;


 int iif;
 __u16 queue_mapping;

 __u16 tc_index;

 __u16 tc_verd;



 __u8 ndisc_nodetype:2;


 __u8 do_not_encrypt:1;




 dma_cookie_t dma_cookie;


 __u32 secmark;


 __u32 mark;

 __u16 vlan_tci;

 sk_buff_data_t transport_header;
 sk_buff_data_t network_header;
 sk_buff_data_t mac_header;

 sk_buff_data_t tail;
 sk_buff_data_t end;
 unsigned char *head,
    *data;
 unsigned int truesize;
 atomic_t users;
};
extern void kfree_skb(struct sk_buff *skb);
extern void __kfree_skb(struct sk_buff *skb);
extern struct sk_buff *__alloc_skb(unsigned int size,
       gfp_t priority, int fclone, int node);
static inline struct sk_buff *alloc_skb(unsigned int size,
     gfp_t priority)
{
 return __alloc_skb(size, priority, 0, -1);
}

static inline struct sk_buff *alloc_skb_fclone(unsigned int size,
            gfp_t priority)
{
 return __alloc_skb(size, priority, 1, -1);
}

extern struct sk_buff *skb_morph(struct sk_buff *dst, struct sk_buff *src);
extern struct sk_buff *skb_clone(struct sk_buff *skb,
     gfp_t priority);
extern struct sk_buff *skb_copy( struct sk_buff *skb,
    gfp_t priority);
extern struct sk_buff *pskb_copy(struct sk_buff *skb,
     gfp_t gfp_mask);
extern int pskb_expand_head(struct sk_buff *skb,
     int nhead, int ntail,
     gfp_t gfp_mask);
extern struct sk_buff *skb_realloc_headroom(struct sk_buff *skb,
         unsigned int headroom);
extern struct sk_buff *skb_copy_expand( struct sk_buff *skb,
           int newheadroom, int newtailroom,
           gfp_t priority);
extern int skb_to_sgvec(struct sk_buff *skb,
        struct scatterlist *sg, int offset,
        int len);
extern int skb_cow_data(struct sk_buff *skb, int tailbits,
        struct sk_buff **trailer);
extern int skb_pad(struct sk_buff *skb, int pad);

extern void skb_over_panic(struct sk_buff *skb, int len,
         void *here);
extern void skb_under_panic(struct sk_buff *skb, int len,
          void *here);

extern int skb_append_datato_frags(struct sock *sk, struct sk_buff *skb,
   int getfrag(void *from, char *to, int offset,
   int len,int odd, struct sk_buff *skb),
   void *from, int length);

struct skb_seq_state
{
 __u32 lower_offset;
 __u32 upper_offset;
 __u32 frag_idx;
 __u32 stepped_offset;
 struct sk_buff *root_skb;
 struct sk_buff *cur_skb;
 __u8 *frag_data;
};

extern void skb_prepare_seq_read(struct sk_buff *skb,
        unsigned int from, unsigned int to,
        struct skb_seq_state *st);
extern unsigned int skb_seq_read(unsigned int consumed, u8 **data,
       struct skb_seq_state *st);
extern void skb_abort_seq_read(struct skb_seq_state *st);

extern unsigned int skb_find_text(struct sk_buff *skb, unsigned int from,
        unsigned int to, struct ts_config *config,
        struct ts_state *state);


static inline unsigned char *skb_end_pointer( struct sk_buff *skb)
{
 return skb->head + skb->end;
}
static inline int skb_queue_empty( struct sk_buff_head *list)
{
 return list->next == (struct sk_buff *)list;
}
static inline struct sk_buff *skb_get(struct sk_buff *skb)
{
 ;
 return skb;
}
static inline int skb_cloned( struct sk_buff *skb)
{
 return skb->cloned &&
        (((&((struct skb_shared_info *)(skb_end_pointer(skb)))->dataref)->counter) & ((1 << 16) - 1)) != 1;
}
static inline int skb_header_cloned( struct sk_buff *skb)
{
 int dataref;

 if (!skb->cloned)
  return 0;

 dataref = ((&((struct skb_shared_info *)(skb_end_pointer(skb)))->dataref)->counter);
 dataref = (dataref & ((1 << 16) - 1)) - (dataref >> 16);
 return dataref != 1;
}
static inline void skb_header_release(struct sk_buff *skb)
{
 ;
 skb->nohdr = 1;
 ;
}
static inline int skb_shared( struct sk_buff *skb)
{
 return ((&skb->users)->counter) != 1;
}
static inline struct sk_buff *skb_share_check(struct sk_buff *skb,
           gfp_t pri)
{
 do { if (pri & (( gfp_t)0x10u)) do { __might_sleep("../../libos/2.6.27/include/linux/skbuff.h", 546); _cond_resched(); } while (0); } while (0);
 if (skb_shared(skb)) {
  struct sk_buff *nskb = skb_clone(skb, pri);
  kfree_skb(skb);
  skb = nskb;
 }
 return skb;
}
static inline struct sk_buff *skb_unshare(struct sk_buff *skb,
       gfp_t pri)
{
 do { if (pri & (( gfp_t)0x10u)) do { __might_sleep("../../libos/2.6.27/include/linux/skbuff.h", 578); _cond_resched(); } while (0); } while (0);
 if (skb_cloned(skb)) {
  struct sk_buff *nskb = skb_copy(skb, pri);
  kfree_skb(skb);
  skb = nskb;
 }
 return skb;
}
static inline struct sk_buff *skb_peek(struct sk_buff_head *list_)
{
 struct sk_buff *list = ((struct sk_buff *)list_)->next;
 if (list == (struct sk_buff *)list_)
  list = 0;
 return list;
}
static inline struct sk_buff *skb_peek_tail(struct sk_buff_head *list_)
{
 struct sk_buff *list = ((struct sk_buff *)list_)->prev;
 if (list == (struct sk_buff *)list_)
  list = 0;
 return list;
}







static inline __u32 skb_queue_len( struct sk_buff_head *list_)
{
 return list_->qlen;
}
static inline void skb_queue_head_init(struct sk_buff_head *list)
{
 ;
 list->prev = list->next = (struct sk_buff *)list;
 list->qlen = 0;
}

static inline void skb_queue_head_init_class(struct sk_buff_head *list,
  struct lock_class_key *_class)
{
 skb_queue_head_init(list);
 do { (void)(_class); } while (0);
}







extern void skb_insert(struct sk_buff *old, struct sk_buff *newsk, struct sk_buff_head *list);
static inline void __skb_insert(struct sk_buff *newsk,
    struct sk_buff *prev, struct sk_buff *next,
    struct sk_buff_head *list)
{
 newsk->next = next;
 newsk->prev = prev;
 next->prev = prev->next = newsk;
 list->qlen++;
}
static inline void __skb_queue_after(struct sk_buff_head *list,
         struct sk_buff *prev,
         struct sk_buff *newsk)
{
 __skb_insert(newsk, prev, prev->next, list);
}

extern void skb_append(struct sk_buff *old, struct sk_buff *newsk,
         struct sk_buff_head *list);

static inline void __skb_queue_before(struct sk_buff_head *list,
          struct sk_buff *next,
          struct sk_buff *newsk)
{
 __skb_insert(newsk, next->prev, next, list);
}
extern void skb_queue_head(struct sk_buff_head *list, struct sk_buff *newsk);
static inline void __skb_queue_head(struct sk_buff_head *list,
        struct sk_buff *newsk)
{
 __skb_queue_after(list, (struct sk_buff *)list, newsk);
}
extern void skb_queue_tail(struct sk_buff_head *list, struct sk_buff *newsk);
static inline void __skb_queue_tail(struct sk_buff_head *list,
       struct sk_buff *newsk)
{
 __skb_queue_before(list, (struct sk_buff *)list, newsk);
}





extern void skb_unlink(struct sk_buff *skb, struct sk_buff_head *list);
static inline void __skb_unlink(struct sk_buff *skb, struct sk_buff_head *list)
{
 struct sk_buff *next, *prev;

 list->qlen--;
 next = skb->next;
 prev = skb->prev;
 skb->next = skb->prev = 0;
 next->prev = prev;
 prev->next = next;
}
extern struct sk_buff *skb_dequeue(struct sk_buff_head *list);
static inline struct sk_buff *__skb_dequeue(struct sk_buff_head *list)
{
 struct sk_buff *skb = skb_peek(list);
 if (skb)
  __skb_unlink(skb, list);
 return skb;
}
extern struct sk_buff *skb_dequeue_tail(struct sk_buff_head *list);
static inline struct sk_buff *__skb_dequeue_tail(struct sk_buff_head *list)
{
 struct sk_buff *skb = skb_peek_tail(list);
 if (skb)
  __skb_unlink(skb, list);
 return skb;
}


static inline int skb_is_nonlinear( struct sk_buff *skb)
{
 return skb->data_len;
}

static inline unsigned int skb_headlen( struct sk_buff *skb)
{
 return skb->len - skb->data_len;
}

static inline int skb_pagelen( struct sk_buff *skb)
{
 int i, len = 0;

 for (i = (int)((struct skb_shared_info *)(skb_end_pointer(skb)))->nr_frags - 1; i >= 0; i--)
  len += ((struct skb_shared_info *)(skb_end_pointer(skb)))->frags[i].size;
 return len + skb_headlen(skb);
}

static inline void skb_fill_page_desc(struct sk_buff *skb, int i,
          struct page *page, int off, int size)
{
 skb_frag_t *frag = &((struct skb_shared_info *)(skb_end_pointer(skb)))->frags[i];

 frag->page = page;
 frag->page_offset = off;
 frag->size = size;
 ((struct skb_shared_info *)(skb_end_pointer(skb)))->nr_frags = i + 1;
}






static inline unsigned char *skb_tail_pointer( struct sk_buff *skb)
{
 return skb->head + skb->tail;
}

static inline void skb_reset_tail_pointer(struct sk_buff *skb)
{
 skb->tail = skb->data - skb->head;
}

static inline void skb_set_tail_pointer(struct sk_buff *skb, int offset)
{
 skb_reset_tail_pointer(skb);
 skb->tail += offset;
}
extern unsigned char *skb_put(struct sk_buff *skb, unsigned int len);
static inline unsigned char *__skb_put(struct sk_buff *skb, unsigned int len)
{
 unsigned char *tmp = skb_tail_pointer(skb);
 ;
 skb->tail += len;
 skb->len += len;
 return tmp;
}

extern unsigned char *skb_push(struct sk_buff *skb, unsigned int len);
static inline unsigned char *__skb_push(struct sk_buff *skb, unsigned int len)
{
 skb->data -= len;
 skb->len += len;
 return skb->data;
}

extern unsigned char *skb_pull(struct sk_buff *skb, unsigned int len);
static inline unsigned char *__skb_pull(struct sk_buff *skb, unsigned int len)
{
 skb->len -= len;
 ;
 return skb->data += len;
}

extern unsigned char *__pskb_pull_tail(struct sk_buff *skb, int delta);

static inline unsigned char *__pskb_pull(struct sk_buff *skb, unsigned int len)
{
 if (len > skb_headlen(skb) &&
     !__pskb_pull_tail(skb, len - skb_headlen(skb)))
  return 0;
 skb->len -= len;
 return skb->data += len;
}

static inline unsigned char *pskb_pull(struct sk_buff *skb, unsigned int len)
{
 return __builtin_expect(!!(len > skb->len), 0) ? 0 : __pskb_pull(skb, len);
}

static inline int pskb_may_pull(struct sk_buff *skb, unsigned int len)
{
 if (__builtin_expect(!!(len <= skb_headlen(skb)), 1))
  return 1;
 if (__builtin_expect(!!(len > skb->len), 0))
  return 0;
 return __pskb_pull_tail(skb, len - skb_headlen(skb)) != 0;
}







static inline unsigned int skb_headroom( struct sk_buff *skb)
{
 return skb->data - skb->head;
}







static inline int skb_tailroom( struct sk_buff *skb)
{
 return skb_is_nonlinear(skb) ? 0 : skb->end - skb->tail;
}
static inline void skb_reserve(struct sk_buff *skb, int len)
{
 skb->data += len;
 skb->tail += len;
}


static inline unsigned char *skb_transport_header( struct sk_buff *skb)
{
 return skb->head + skb->transport_header;
}

static inline void skb_reset_transport_header(struct sk_buff *skb)
{
 skb->transport_header = skb->data - skb->head;
}

static inline void skb_set_transport_header(struct sk_buff *skb,
         int offset)
{
 skb_reset_transport_header(skb);
 skb->transport_header += offset;
}

static inline unsigned char *skb_network_header( struct sk_buff *skb)
{
 return skb->head + skb->network_header;
}

static inline void skb_reset_network_header(struct sk_buff *skb)
{
 skb->network_header = skb->data - skb->head;
}

static inline void skb_set_network_header(struct sk_buff *skb, int offset)
{
 skb_reset_network_header(skb);
 skb->network_header += offset;
}

static inline unsigned char *skb_mac_header( struct sk_buff *skb)
{
 return skb->head + skb->mac_header;
}

static inline int skb_mac_header_was_set( struct sk_buff *skb)
{
 return skb->mac_header != ~0U;
}

static inline void skb_reset_mac_header(struct sk_buff *skb)
{
 skb->mac_header = skb->data - skb->head;
}

static inline void skb_set_mac_header(struct sk_buff *skb, int offset)
{
 skb_reset_mac_header(skb);
 skb->mac_header += offset;
}
static inline int skb_transport_offset( struct sk_buff *skb)
{
 return skb_transport_header(skb) - skb->data;
}

static inline u32 skb_network_header_len( struct sk_buff *skb)
{
 return skb->transport_header - skb->network_header;
}

static inline int skb_network_offset( struct sk_buff *skb)
{
 return skb_network_header(skb) - skb->data;
}
extern int ___pskb_trim(struct sk_buff *skb, unsigned int len);

static inline void __skb_trim(struct sk_buff *skb, unsigned int len)
{
 if (__builtin_expect(!!(skb->data_len), 0)) {
  ;
  return;
 }
 skb->len = len;
 skb_set_tail_pointer(skb, len);
}

extern void skb_trim(struct sk_buff *skb, unsigned int len);

static inline int __pskb_trim(struct sk_buff *skb, unsigned int len)
{
 if (skb->data_len)
  return ___pskb_trim(skb, len);
 __skb_trim(skb, len);
 return 0;
}

static inline int pskb_trim(struct sk_buff *skb, unsigned int len)
{
 return (len < skb->len) ? __pskb_trim(skb, len) : 0;
}
static inline void pskb_trim_unique(struct sk_buff *skb, unsigned int len)
{
 int err = pskb_trim(skb, len);
 ;
}
static inline void skb_orphan(struct sk_buff *skb)
{
 if (skb->destructor)
  skb->destructor(skb);
 skb->destructor = 0;
 skb->sk = 0;
}
extern void skb_queue_purge(struct sk_buff_head *list);
static inline void __skb_queue_purge(struct sk_buff_head *list)
{
 struct sk_buff *skb;
 while ((skb = __skb_dequeue(list)) != 0)
  kfree_skb(skb);
}
static inline struct sk_buff *__dev_alloc_skb(unsigned int length,
           gfp_t gfp_mask)
{
 struct sk_buff *skb = alloc_skb(length + 16, gfp_mask);
 if (__builtin_expect(!!(skb), 1))
  skb_reserve(skb, 16);
 return skb;
}

extern struct sk_buff *dev_alloc_skb(unsigned int length);

extern struct sk_buff *__netdev_alloc_skb(struct net_device *dev,
  unsigned int length, gfp_t gfp_mask);
static inline struct sk_buff *netdev_alloc_skb(struct net_device *dev,
  unsigned int length)
{
 return __netdev_alloc_skb(dev, length, ((( gfp_t)0x20u)));
}
static inline int skb_clone_writable(struct sk_buff *skb, unsigned int len)
{
 return !skb_header_cloned(skb) &&
        skb_headroom(skb) + len <= skb->hdr_len;
}

static inline int __skb_cow(struct sk_buff *skb, unsigned int headroom,
       int cloned)
{
 int delta = 0;

 if (headroom < 16)
  headroom = 16;
 if (headroom > skb_headroom(skb))
  delta = headroom - skb_headroom(skb);

 if (delta || cloned)
  return pskb_expand_head(skb, (((delta)+((typeof(delta))(16)-1))&~((typeof(delta))(16)-1)), 0,
     ((( gfp_t)0x20u)));
 return 0;
}
static inline int skb_cow(struct sk_buff *skb, unsigned int headroom)
{
 return __skb_cow(skb, headroom, skb_cloned(skb));
}
static inline int skb_cow_head(struct sk_buff *skb, unsigned int headroom)
{
 return __skb_cow(skb, headroom, skb_header_cloned(skb));
}
static inline int skb_padto(struct sk_buff *skb, unsigned int len)
{
 unsigned int size = skb->len;
 if (__builtin_expect(!!(size >= len), 1))
  return 0;
 return skb_pad(skb, len - size);
}

static inline int skb_add_data(struct sk_buff *skb,
          char *from, int copy)
{
 int off = skb->len;

 if (skb->ip_summed == 0) {
  int err = 0;
  __wsum csum = csum_partial_copy_from_user(from, skb_put(skb, copy),
           copy, 0, &err);
  if (!err) {
   skb->csum = csum_block_add(skb->csum, csum, off);
   return 0;
  }
 } else if (!copy_from_user(skb_put(skb, copy), from, copy))
  return 0;

 __skb_trim(skb, off);
 return -14;
}

static inline int skb_can_coalesce(struct sk_buff *skb, int i,
       struct page *page, int off)
{
 if (i) {
  struct skb_frag_struct *frag = &((struct skb_shared_info *)(skb_end_pointer(skb)))->frags[i - 1];

  return page == frag->page &&
         off == frag->page_offset + frag->size;
 }
 return 0;
}

static inline int __skb_linearize(struct sk_buff *skb)
{
 return __pskb_pull_tail(skb, skb->data_len) ? 0 : -12;
}
static inline int skb_linearize(struct sk_buff *skb)
{
 return skb_is_nonlinear(skb) ? __skb_linearize(skb) : 0;
}
static inline int skb_linearize_cow(struct sk_buff *skb)
{
 return skb_is_nonlinear(skb) || skb_cloned(skb) ?
        __skb_linearize(skb) : 0;
}
static inline void skb_postpull_rcsum(struct sk_buff *skb,
          void *start, unsigned int len)
{
 if (skb->ip_summed == 2)
  skb->csum = csum_sub(skb->csum, csum_partial(start, len, 0));
}

unsigned char *skb_pull_rcsum(struct sk_buff *skb, unsigned int len);
static inline int pskb_trim_rcsum(struct sk_buff *skb, unsigned int len)
{
 if (__builtin_expect(!!(len >= skb->len), 1))
  return 0;
 if (skb->ip_summed == 2)
  skb->ip_summed = 0;
 return __pskb_trim(skb, len);
}
extern struct sk_buff *__skb_recv_datagram(struct sock *sk, unsigned flags,
        int *peeked, int *err);
extern struct sk_buff *skb_recv_datagram(struct sock *sk, unsigned flags,
      int noblock, int *err);
extern unsigned int datagram_poll(struct file *file, struct socket *sock,
         struct poll_table_struct *wait);
extern int skb_copy_datagram_iovec( struct sk_buff *from,
            int offset, struct iovec *to,
            int size);
extern int skb_copy_and_csum_datagram_iovec(struct sk_buff *skb,
       int hlen,
       struct iovec *iov);
extern int skb_copy_datagram_from_iovec(struct sk_buff *skb,
          int offset,
          struct iovec *from,
          int len);
extern void skb_free_datagram(struct sock *sk, struct sk_buff *skb);
extern int skb_kill_datagram(struct sock *sk, struct sk_buff *skb,
      unsigned int flags);
extern __wsum skb_checksum( struct sk_buff *skb, int offset,
        int len, __wsum csum);
extern int skb_copy_bits( struct sk_buff *skb, int offset,
         void *to, int len);
extern int skb_store_bits(struct sk_buff *skb, int offset,
          void *from, int len);
extern __wsum skb_copy_and_csum_bits( struct sk_buff *skb,
           int offset, u8 *to, int len,
           __wsum csum);
extern int skb_splice_bits(struct sk_buff *skb,
      unsigned int offset,
      struct pipe_inode_info *pipe,
      unsigned int len,
      unsigned int flags);
extern void skb_copy_and_csum_dev( struct sk_buff *skb, u8 *to);
extern void skb_split(struct sk_buff *skb,
     struct sk_buff *skb1, u32 len);

extern struct sk_buff *skb_segment(struct sk_buff *skb, int features);

static inline void *skb_header_pointer( struct sk_buff *skb, int offset,
           int len, void *buffer)
{
 int hlen = skb_headlen(skb);

 if (hlen - offset >= len)
  return skb->data + offset;

 if (skb_copy_bits(skb, offset, buffer, len) < 0)
  return 0;

 return buffer;
}

static inline void skb_copy_from_linear_data( struct sk_buff *skb,
          void *to,
          unsigned int len)
{
 memcpy(to, skb->data, len);
}

static inline void skb_copy_from_linear_data_offset( struct sk_buff *skb,
          int offset, void *to,
          unsigned int len)
{
 memcpy(to, skb->data + offset, len);
}

static inline void skb_copy_to_linear_data(struct sk_buff *skb,
        void *from,
        unsigned int len)
{
 memcpy(skb->data, from, len);
}

static inline void skb_copy_to_linear_data_offset(struct sk_buff *skb,
        int offset,
        void *from,
        unsigned int len)
{
 memcpy(skb->data + offset, from, len);
}

extern void skb_init(void);
static inline void skb_get_timestamp( struct sk_buff *skb, struct timeval *stamp)
{
 *stamp = ns_to_timeval((skb->tstamp).tv64);
}

static inline void __net_timestamp(struct sk_buff *skb)
{
 skb->tstamp = ktime_get_real();
}

static inline ktime_t net_timedelta(ktime_t t)
{
 return ({ ({ ktime_t var; var.tv64 = (ktime_get_real()).tv64 - (t).tv64; var; }); });
}

static inline ktime_t net_invalid_timestamp(void)
{
 return ktime_set(0, 0);
}

extern __sum16 __skb_checksum_complete_head(struct sk_buff *skb, int len);
extern __sum16 __skb_checksum_complete(struct sk_buff *skb);

static inline int skb_csum_unnecessary( struct sk_buff *skb)
{
 return skb->ip_summed & 1;
}
static inline __sum16 skb_checksum_complete(struct sk_buff *skb)
{
 return skb_csum_unnecessary(skb) ?
        0 : __skb_checksum_complete(skb);
}


extern void nf_conntrack_destroy(struct nf_conntrack *nfct);
static inline void nf_conntrack_put(struct nf_conntrack *nfct)
{
 if (nfct && 1)
  nf_conntrack_destroy(nfct);
}
static inline void nf_conntrack_get(struct nf_conntrack *nfct)
{
 if (nfct)
  ;
}
static inline void nf_conntrack_get_reasm(struct sk_buff *skb)
{
 if (skb)
  ;
}
static inline void nf_conntrack_put_reasm(struct sk_buff *skb)
{
 if (skb)
  kfree_skb(skb);
}


static inline void nf_bridge_put(struct nf_bridge_info *nf_bridge)
{
 if (nf_bridge && 1)
  kfree(nf_bridge);
}
static inline void nf_bridge_get(struct nf_bridge_info *nf_bridge)
{
 if (nf_bridge)
  ;
}

static inline void nf_reset(struct sk_buff *skb)
{

 nf_conntrack_put(skb->nfct);
 skb->nfct = 0;
 nf_conntrack_put_reasm(skb->nfct_reasm);
 skb->nfct_reasm = 0;


 nf_bridge_put(skb->nf_bridge);
 skb->nf_bridge = 0;

}


static inline void __nf_copy(struct sk_buff *dst, struct sk_buff *src)
{

 dst->nfct = src->nfct;
 nf_conntrack_get(src->nfct);
 dst->nfctinfo = src->nfctinfo;
 dst->nfct_reasm = src->nfct_reasm;
 nf_conntrack_get_reasm(src->nfct_reasm);


 dst->nf_bridge = src->nf_bridge;
 nf_bridge_get(src->nf_bridge);

}

static inline void nf_copy(struct sk_buff *dst, struct sk_buff *src)
{

 nf_conntrack_put(dst->nfct);
 nf_conntrack_put_reasm(dst->nfct_reasm);


 nf_bridge_put(dst->nf_bridge);

 __nf_copy(dst, src);
}


static inline void skb_copy_secmark(struct sk_buff *to, struct sk_buff *from)
{
 to->secmark = from->secmark;
}

static inline void skb_init_secmark(struct sk_buff *skb)
{
 skb->secmark = 0;
}
static inline void skb_set_queue_mapping(struct sk_buff *skb, u16 queue_mapping)
{
 skb->queue_mapping = queue_mapping;
}

static inline u16 skb_get_queue_mapping(struct sk_buff *skb)
{
 return skb->queue_mapping;
}

static inline void skb_copy_queue_mapping(struct sk_buff *to, struct sk_buff *from)
{
 to->queue_mapping = from->queue_mapping;
}

static inline int skb_is_gso( struct sk_buff *skb)
{
 return ((struct skb_shared_info *)(skb_end_pointer(skb)))->gso_size;
}

static inline int skb_is_gso_v6( struct sk_buff *skb)
{
 return ((struct skb_shared_info *)(skb_end_pointer(skb)))->gso_type & SKB_GSO_TCPV6;
}

extern void __skb_warn_lro_forwarding( struct sk_buff *skb);

static inline bool skb_warn_if_lro( struct sk_buff *skb)
{


 struct skb_shared_info *shinfo = ((struct skb_shared_info *)(skb_end_pointer(skb)));
 if (shinfo->gso_size != 0 && __builtin_expect(!!(shinfo->gso_type == 0), 0)) {
  __skb_warn_lro_forwarding(skb);
  return _true;
 }
 return _false;
}

static inline void skb_forward_csum(struct sk_buff *skb)
{

 if (skb->ip_summed == 2)
  skb->ip_summed = 0;
}

bool skb_partial_csum_set(struct sk_buff *skb, u16 start, u16 off);

static inline struct nlmsghdr *nlmsg_hdr( struct sk_buff *skb)
{
 return (struct nlmsghdr *)skb->data;
}

struct netlink_skb_parms
{
 struct ucred creds;
 __u32 pid;
 __u32 dst_group;
 kernel_cap_t eff_cap;
 __u32 loginuid;
 __u32 sessionid;
 __u32 sid;
};





extern struct sock *netlink_kernel_create(struct net *net,
       int unit,unsigned int groups,
       void (*input)(struct sk_buff *skb),
       struct mutex *cb_mutex,
       struct module *module);
extern void netlink_kernel_release(struct sock *sk);
extern int netlink_change_ngroups(struct sock *sk, unsigned int groups);
extern void netlink_clear_multicast_users(struct sock *sk, unsigned int group);
extern void netlink_ack(struct sk_buff *in_skb, struct nlmsghdr *nlh, int err);
extern int netlink_has_listeners(struct sock *sk, unsigned int group);
extern int netlink_unicast(struct sock *ssk, struct sk_buff *skb, __u32 pid, int nonblock);
extern int netlink_broadcast(struct sock *ssk, struct sk_buff *skb, __u32 pid,
        __u32 group, gfp_t allocation);
extern void netlink_set_err(struct sock *ssk, __u32 pid, __u32 group, int code);
extern int netlink_register_notifier(struct notifier_block *nb);
extern int netlink_unregister_notifier(struct notifier_block *nb);


struct sock *netlink_getsockbyfilp(struct file *filp);
int netlink_attachskb(struct sock *sk, struct sk_buff *skb,
        long *timeo, struct sock *ssk);
void netlink_detachskb(struct sock *sk, struct sk_buff *skb);
int netlink_sendskb(struct sock *sk, struct sk_buff *skb);
struct netlink_callback
{
 struct sk_buff *skb;
 struct nlmsghdr *nlh;
 int (*dump)(struct sk_buff * skb, struct netlink_callback *cb);
 int (*done)(struct netlink_callback *cb);
 int family;
 long args[6];
};

struct netlink_notify
{
 struct net *net;
 int pid;
 int protocol;
};

static __inline__ struct nlmsghdr *
__nlmsg_put(struct sk_buff *skb, u32 pid, u32 seq, int type, int len, int flags)
{
 struct nlmsghdr *nlh;
 int size = ((len)+( ((((int) ( ((sizeof(struct nlmsghdr))+4 -1) & ~(4 -1) )))+4 -1) & ~(4 -1) ));

 nlh = (struct nlmsghdr*)skb_put(skb, ( ((size)+4 -1) & ~(4 -1) ));
 nlh->nlmsg_type = type;
 nlh->nlmsg_len = size;
 nlh->nlmsg_flags = flags;
 nlh->nlmsg_pid = pid;
 nlh->nlmsg_seq = seq;
 memset(((void*)(((char*)nlh) + ((0)+( ((((int) ( ((sizeof(struct nlmsghdr))+4 -1) & ~(4 -1) )))+4 -1) & ~(4 -1) )))) + len, 0, ( ((size)+4 -1) & ~(4 -1) ) - size);
 return nlh;
}
extern int netlink_dump_start(struct sock *ssk, struct sk_buff *skb,
         struct nlmsghdr *nlh,
         int (*dump)(struct sk_buff *skb, struct netlink_callback*),
         int (*done)(struct netlink_callback*));




extern void netlink_set_nonroot(int protocol, unsigned flag);






struct genlmsghdr {
 __u8 cmd;
 __u8 version;
 __u16 reserved;
};
enum {
 CTRL_CMD_UNSPEC,
 CTRL_CMD_NEWFAMILY,
 CTRL_CMD_DELFAMILY,
 CTRL_CMD_GETFAMILY,
 CTRL_CMD_NEWOPS,
 CTRL_CMD_DELOPS,
 CTRL_CMD_GETOPS,
 CTRL_CMD_NEWMCAST_GRP,
 CTRL_CMD_DELMCAST_GRP,
 CTRL_CMD_GETMCAST_GRP,
 __CTRL_CMD_MAX,
};



enum {
 CTRL_ATTR_UNSPEC,
 CTRL_ATTR_FAMILY_ID,
 CTRL_ATTR_FAMILY_NAME,
 CTRL_ATTR_VERSION,
 CTRL_ATTR_HDRSIZE,
 CTRL_ATTR_MAXATTR,
 CTRL_ATTR_OPS,
 CTRL_ATTR_MCAST_GROUPS,
 __CTRL_ATTR_MAX,
};



enum {
 CTRL_ATTR_OP_UNSPEC,
 CTRL_ATTR_OP_ID,
 CTRL_ATTR_OP_FLAGS,
 __CTRL_ATTR_OP_MAX,
};



enum {
 CTRL_ATTR_MCAST_GRP_UNSPEC,
 CTRL_ATTR_MCAST_GRP_NAME,
 CTRL_ATTR_MCAST_GRP_ID,
 __CTRL_ATTR_MCAST_GRP_MAX,
};
enum {
 NLA_UNSPEC,
 NLA_U8,
 NLA_U16,
 NLA_U32,
 NLA_U64,
 NLA_STRING,
 NLA_FLAG,
 NLA_MSECS,
 NLA_NESTED,
 NLA_NESTED_COMPAT,
 NLA_NUL_STRING,
 NLA_BINARY,
 __NLA_TYPE_MAX,
};
struct nla_policy {
 u16 type;
 u16 len;
};






struct nl_info {
 struct nlmsghdr *nlh;
 struct net *nl_net;
 u32 pid;
};

extern int netlink_rcv_skb(struct sk_buff *skb,
     int (*cb)(struct sk_buff *,
        struct nlmsghdr *));
extern int nlmsg_notify(struct sock *sk, struct sk_buff *skb,
         u32 pid, unsigned int group, int report,
         gfp_t flags);

extern int nla_validate(struct nlattr *head, int len, int maxtype,
         struct nla_policy *policy);
extern int nla_parse(struct nlattr *tb[], int maxtype,
      struct nlattr *head, int len,
      struct nla_policy *policy);
extern struct nlattr * nla_find(struct nlattr *head, int len, int attrtype);
extern size_t nla_strlcpy(char *dst, struct nlattr *nla,
        size_t dstsize);
extern int nla_memcpy(void *dest, struct nlattr *src, int count);
extern int nla_memcmp( struct nlattr *nla, void *data,
       size_t size);
extern int nla_strcmp( struct nlattr *nla, char *str);
extern struct nlattr * __nla_reserve(struct sk_buff *skb, int attrtype,
          int attrlen);
extern void * __nla_reserve_nohdr(struct sk_buff *skb, int attrlen);
extern struct nlattr * nla_reserve(struct sk_buff *skb, int attrtype,
        int attrlen);
extern void * nla_reserve_nohdr(struct sk_buff *skb, int attrlen);
extern void __nla_put(struct sk_buff *skb, int attrtype,
      int attrlen, void *data);
extern void __nla_put_nohdr(struct sk_buff *skb, int attrlen,
     void *data);
extern int nla_put(struct sk_buff *skb, int attrtype,
    int attrlen, void *data);
extern int nla_put_nohdr(struct sk_buff *skb, int attrlen,
          void *data);
extern int nla_append(struct sk_buff *skb, int attrlen,
       void *data);
static inline int nlmsg_msg_size(int payload)
{
 return ((int) ( ((sizeof(struct nlmsghdr))+4 -1) & ~(4 -1) )) + payload;
}





static inline int nlmsg_total_size(int payload)
{
 return ( ((nlmsg_msg_size(payload))+4 -1) & ~(4 -1) );
}





static inline int nlmsg_padlen(int payload)
{
 return nlmsg_total_size(payload) - nlmsg_msg_size(payload);
}





static inline void *nlmsg_data( struct nlmsghdr *nlh)
{
 return (unsigned char *) nlh + ((int) ( ((sizeof(struct nlmsghdr))+4 -1) & ~(4 -1) ));
}





static inline int nlmsg_len( struct nlmsghdr *nlh)
{
 return nlh->nlmsg_len - ((int) ( ((sizeof(struct nlmsghdr))+4 -1) & ~(4 -1) ));
}






static inline struct nlattr *nlmsg_attrdata( struct nlmsghdr *nlh,
         int hdrlen)
{
 unsigned char *data = nlmsg_data(nlh);
 return (struct nlattr *) (data + ( ((hdrlen)+4 -1) & ~(4 -1) ));
}






static inline int nlmsg_attrlen( struct nlmsghdr *nlh, int hdrlen)
{
 return nlmsg_len(nlh) - ( ((hdrlen)+4 -1) & ~(4 -1) );
}






static inline int nlmsg_ok( struct nlmsghdr *nlh, int remaining)
{
 return (remaining >= sizeof(struct nlmsghdr) &&
  nlh->nlmsg_len >= sizeof(struct nlmsghdr) &&
  nlh->nlmsg_len <= remaining);
}
static inline struct nlmsghdr *nlmsg_next(struct nlmsghdr *nlh, int *remaining)
{
 int totlen = ( ((nlh->nlmsg_len)+4 -1) & ~(4 -1) );

 *remaining -= totlen;

 return (struct nlmsghdr *) ((unsigned char *) nlh + totlen);
}
static inline int nlmsg_parse(struct nlmsghdr *nlh, int hdrlen,
         struct nlattr *tb[], int maxtype,
         struct nla_policy *policy)
{
 if (nlh->nlmsg_len < nlmsg_msg_size(hdrlen))
  return -22;

 return nla_parse(tb, maxtype, nlmsg_attrdata(nlh, hdrlen),
    nlmsg_attrlen(nlh, hdrlen), policy);
}
static inline struct nlattr *nlmsg_find_attr(struct nlmsghdr *nlh,
          int hdrlen, int attrtype)
{
 return nla_find(nlmsg_attrdata(nlh, hdrlen),
   nlmsg_attrlen(nlh, hdrlen), attrtype);
}
static inline int nlmsg_validate(struct nlmsghdr *nlh, int hdrlen, int maxtype,
     struct nla_policy *policy)
{
 if (nlh->nlmsg_len < nlmsg_msg_size(hdrlen))
  return -22;

 return nla_validate(nlmsg_attrdata(nlh, hdrlen),
       nlmsg_attrlen(nlh, hdrlen), maxtype, policy);
}







static inline int nlmsg_report(struct nlmsghdr *nlh)
{
 return !!(nlh->nlmsg_flags & 8);
}
static inline struct nlmsghdr *nlmsg_put(struct sk_buff *skb, u32 pid, u32 seq,
      int type, int payload, int flags)
{
 if (__builtin_expect(!!(skb_tailroom(skb) < nlmsg_total_size(payload)), 0))
  return 0;

 return __nlmsg_put(skb, pid, seq, type, payload, flags);
}
static inline struct nlmsghdr *nlmsg_put_answer(struct sk_buff *skb,
      struct netlink_callback *cb,
      int type, int payload,
      int flags)
{
 return nlmsg_put(skb, (*(struct netlink_skb_parms*)&((cb->skb)->cb)).pid, cb->nlh->nlmsg_seq,
    type, payload, flags);
}
static inline struct sk_buff *nlmsg_new(size_t payload, gfp_t flags)
{
 return alloc_skb(nlmsg_total_size(payload), flags);
}
static inline int nlmsg_end(struct sk_buff *skb, struct nlmsghdr *nlh)
{
 nlh->nlmsg_len = skb_tail_pointer(skb) - (unsigned char *)nlh;

 return skb->len;
}







static inline void *nlmsg_get_pos(struct sk_buff *skb)
{
 return skb_tail_pointer(skb);
}
static inline void nlmsg_trim(struct sk_buff *skb, void *mark)
{
 if (mark)
  skb_trim(skb, (unsigned char *) mark - skb->data);
}
static inline void nlmsg_cancel(struct sk_buff *skb, struct nlmsghdr *nlh)
{
 nlmsg_trim(skb, nlh);
}





static inline void nlmsg_free(struct sk_buff *skb)
{
 kfree_skb(skb);
}
static inline int nlmsg_multicast(struct sock *sk, struct sk_buff *skb,
      u32 pid, unsigned int group, gfp_t flags)
{
 int err;

 (*(struct netlink_skb_parms*)&((skb)->cb)).dst_group = group;

 err = netlink_broadcast(sk, skb, pid, group, flags);
 if (err > 0)
  err = 0;

 return err;
}







static inline int nlmsg_unicast(struct sock *sk, struct sk_buff *skb, u32 pid)
{
 int err;

 err = netlink_unicast(sk, skb, pid, 0x40);
 if (err > 0)
  err = 0;

 return err;
}
static inline int nla_attr_size(int payload)
{
 return ((int) (((sizeof(struct nlattr)) + 4 - 1) & ~(4 - 1))) + payload;
}





static inline int nla_total_size(int payload)
{
 return (((nla_attr_size(payload)) + 4 - 1) & ~(4 - 1));
}





static inline int nla_padlen(int payload)
{
 return nla_total_size(payload) - nla_attr_size(payload);
}





static inline int nla_type( struct nlattr *nla)
{
 return nla->nla_type & ~((1 << 15) | (1 << 14));
}





static inline struct nlattr *nla_data( struct nlattr *nla)
{
        return (struct nlattr *)((char *) nla + ((int) (((sizeof(struct nlattr)) + 4 - 1) & ~(4 - 1))));
}





static inline int nla_len( struct nlattr *nla)
{
 return nla->nla_len - ((int) (((sizeof(struct nlattr)) + 4 - 1) & ~(4 - 1)));
}






static inline int nla_ok( struct nlattr *nla, int remaining)
{
 return remaining >= (int) sizeof(*nla) &&
        nla->nla_len >= sizeof(*nla) &&
        nla->nla_len <= remaining;
}
static inline struct nlattr *nla_next( struct nlattr *nla, int *remaining)
{
 int totlen = (((nla->nla_len) + 4 - 1) & ~(4 - 1));

 *remaining -= totlen;
 return (struct nlattr *) ((char *) nla + totlen);
}
static inline struct nlattr *nla_find_nested(struct nlattr *nla, int attrtype)
{
 return nla_find(nla_data(nla), nla_len(nla), attrtype);
}
static inline int nla_parse_nested(struct nlattr *tb[], int maxtype,
       struct nlattr *nla,
       struct nla_policy *policy)
{
 return nla_parse(tb, maxtype, nla_data(nla), nla_len(nla), policy);
}
static inline int __nla_parse_nested_compat(struct nlattr *tb[], int maxtype,
         struct nlattr *nla,
         struct nla_policy *policy,
         int len)
{
 int nested_len = nla_len(nla) - (((len) + 4 - 1) & ~(4 - 1));

 if (nested_len < 0)
  return -22;
 if (nested_len >= nla_attr_size(0))
  return nla_parse(tb, maxtype, nla_data(nla) + (((len) + 4 - 1) & ~(4 - 1)),
     nested_len, policy);
 memset(tb, 0, sizeof(struct nlattr *) * (maxtype + 1));
 return 0;
}
static inline int nla_put_u8(struct sk_buff *skb, int attrtype, u8 value)
{
 return nla_put(skb, attrtype, sizeof(u8), &value);
}







static inline int nla_put_u16(struct sk_buff *skb, int attrtype, u16 value)
{
 return nla_put(skb, attrtype, sizeof(u16), &value);
}







static inline int nla_put_u32(struct sk_buff *skb, int attrtype, u32 value)
{
 return nla_put(skb, attrtype, sizeof(u32), &value);
}







static inline int nla_put_u64(struct sk_buff *skb, int attrtype, u64 value)
{
 return nla_put(skb, attrtype, sizeof(u64), &value);
}







static inline int nla_put_string(struct sk_buff *skb, int attrtype,
     char *str)
{
 return nla_put(skb, attrtype, strlen(str) + 1, str);
}






static inline int nla_put_flag(struct sk_buff *skb, int attrtype)
{
 return nla_put(skb, attrtype, 0, 0);
}







static inline int nla_put_msecs(struct sk_buff *skb, int attrtype,
    unsigned long jiffies)
{
 u64 tmp = jiffies_to_msecs(jiffies);
 return nla_put(skb, attrtype, sizeof(u64), &tmp);
}
static inline u32 nla_get_u32(struct nlattr *nla)
{
 return *(u32 *) nla_data(nla);
}





static inline __be32 nla_get_be32(struct nlattr *nla)
{
 return *(__be32 *) nla_data(nla);
}





static inline u16 nla_get_u16(struct nlattr *nla)
{
 return *(u16 *) nla_data(nla);
}





static inline __be16 nla_get_be16(struct nlattr *nla)
{
 return *(__be16 *) nla_data(nla);
}





static inline __le16 nla_get_le16(struct nlattr *nla)
{
 return *(__le16 *) nla_data(nla);
}





static inline u8 nla_get_u8(struct nlattr *nla)
{
 return *(u8 *) nla_data(nla);
}





static inline u64 nla_get_u64(struct nlattr *nla)
{
 u64 tmp;

 nla_memcpy(&tmp, nla, sizeof(tmp));

 return tmp;
}





static inline int nla_get_flag(struct nlattr *nla)
{
 return !!nla;
}







static inline unsigned long nla_get_msecs(struct nlattr *nla)
{
 u64 msecs = nla_get_u64(nla);

 return msecs_to_jiffies((unsigned long) msecs);
}
static inline struct nlattr *nla_nest_start(struct sk_buff *skb, int attrtype)
{
 struct nlattr *start = (struct nlattr *)skb_tail_pointer(skb);

 if (nla_put(skb, attrtype, 0, 0) < 0)
  return 0;

 return start;
}
static inline int nla_nest_end(struct sk_buff *skb, struct nlattr *start)
{
 start->nla_len = skb_tail_pointer(skb) - (unsigned char *)start;
 return skb->len;
}
static inline struct nlattr *nla_nest_compat_start(struct sk_buff *skb,
         int attrtype, int attrlen,
         void *data)
{
 struct nlattr *start = (struct nlattr *)skb_tail_pointer(skb);

 if (nla_put(skb, attrtype, attrlen, data) < 0)
  return 0;
 if (nla_nest_start(skb, attrtype) == 0) {
  nlmsg_trim(skb, start);
  return 0;
 }
 return start;
}
static inline int nla_nest_compat_end(struct sk_buff *skb, struct nlattr *start)
{
 struct nlattr *nest = (void *)start + ( ((start->nla_len)+4 -1) & ~(4 -1) );

 start->nla_len = skb_tail_pointer(skb) - (unsigned char *)start;
 return nla_nest_end(skb, nest);
}
static inline void nla_nest_cancel(struct sk_buff *skb, struct nlattr *start)
{
 nlmsg_trim(skb, start);
}
static inline int nla_validate_nested(struct nlattr *start, int maxtype,
          struct nla_policy *policy)
{
 return nla_validate(nla_data(start), nla_len(start), maxtype, policy);
}
struct genl_multicast_group
{
 struct genl_family *family;
 struct list_head list;
 char name[16];
 u32 id;
};
struct genl_family
{
 unsigned int id;
 unsigned int hdrsize;
 char name[16];
 unsigned int version;
 unsigned int maxattr;
 struct nlattr ** attrbuf;
 struct list_head ops_list;
 struct list_head family_list;
 struct list_head mcast_groups;
};
struct genl_info
{
 u32 snd_seq;
 u32 snd_pid;
 struct nlmsghdr * nlhdr;
 struct genlmsghdr * genlhdr;
 void * userhdr;
 struct nlattr ** attrs;
};
struct genl_ops
{
 u8 cmd;
 unsigned int flags;
 struct nla_policy *policy;
 int (*doit)(struct sk_buff *skb,
           struct genl_info *info);
 int (*dumpit)(struct sk_buff *skb,
      struct netlink_callback *cb);
 int (*done)(struct netlink_callback *cb);
 struct list_head ops_list;
};

extern int genl_register_family(struct genl_family *family);
extern int genl_unregister_family(struct genl_family *family);
extern int genl_register_ops(struct genl_family *, struct genl_ops *ops);
extern int genl_unregister_ops(struct genl_family *, struct genl_ops *ops);
extern int genl_register_mc_group(struct genl_family *family,
      struct genl_multicast_group *grp);
extern void genl_unregister_mc_group(struct genl_family *family,
         struct genl_multicast_group *grp);

extern struct sock *genl_sock;
static inline void *genlmsg_put(struct sk_buff *skb, u32 pid, u32 seq,
    struct genl_family *family, int flags, u8 cmd)
{
 struct nlmsghdr *nlh;
 struct genlmsghdr *hdr;

 nlh = nlmsg_put(skb, pid, seq, family->id, ( ((sizeof(struct genlmsghdr))+4 -1) & ~(4 -1) ) +
   family->hdrsize, flags);
 if (nlh == 0)
  return 0;

 hdr = nlmsg_data(nlh);
 hdr->cmd = cmd;
 hdr->version = family->version;
 hdr->reserved = 0;

 return (char *) hdr + ( ((sizeof(struct genlmsghdr))+4 -1) & ~(4 -1) );
}
static inline void *genlmsg_put_reply(struct sk_buff *skb,
          struct genl_info *info,
          struct genl_family *family,
          int flags, u8 cmd)
{
 return genlmsg_put(skb, info->snd_pid, info->snd_seq, family,
      flags, cmd);
}






static inline int genlmsg_end(struct sk_buff *skb, void *hdr)
{
        return nlmsg_end(skb, (struct nlmsghdr *)(hdr - ( ((sizeof(struct genlmsghdr))+4 -1) & ~(4 -1) ) - ((int) ( ((sizeof(struct nlmsghdr))+4 -1) & ~(4 -1) ))));
}






static inline void genlmsg_cancel(struct sk_buff *skb, void *hdr)
{
        nlmsg_cancel(skb, (struct nlmsghdr *)hdr - ( ((sizeof(struct genlmsghdr))+4 -1) & ~(4 -1) ) - ((int) ( ((sizeof(struct nlmsghdr))+4 -1) & ~(4 -1) )));
}
static inline int genlmsg_multicast(struct sk_buff *skb, u32 pid,
        unsigned int group, gfp_t flags)
{
 return nlmsg_multicast(genl_sock, skb, pid, group, flags);
}






static inline int genlmsg_unicast(struct sk_buff *skb, u32 pid)
{
 return nlmsg_unicast(genl_sock, skb, pid);
}






static inline int genlmsg_reply(struct sk_buff *skb, struct genl_info *info)
{
 return genlmsg_unicast(skb, info->snd_pid);
}





static inline void *genlmsg_data( struct genlmsghdr *gnlh)
{
 return ((unsigned char *) gnlh + ( ((sizeof(struct genlmsghdr))+4 -1) & ~(4 -1) ));
}





static inline int genlmsg_len( struct genlmsghdr *gnlh)
{
 struct nlmsghdr *nlh = (struct nlmsghdr *)((unsigned char *)gnlh -
       ((int) ( ((sizeof(struct nlmsghdr))+4 -1) & ~(4 -1) )));
 return (nlh->nlmsg_len - ( ((sizeof(struct genlmsghdr))+4 -1) & ~(4 -1) ) - ((int) ( ((sizeof(struct nlmsghdr))+4 -1) & ~(4 -1) )));
}





static inline int genlmsg_msg_size(int payload)
{
 return ( ((sizeof(struct genlmsghdr))+4 -1) & ~(4 -1) ) + payload;
}





static inline int genlmsg_total_size(int payload)
{
 return ( ((genlmsg_msg_size(payload))+4 -1) & ~(4 -1) );
}






static inline struct sk_buff *genlmsg_new(size_t payload, gfp_t flags)
{
 return nlmsg_new(genlmsg_total_size(payload), flags);
}


extern struct kmem_cache *taskstats_cache;
extern struct mutex taskstats_exit_mutex;

static inline void taskstats_tgid_init(struct signal_struct *sig)
{
 sig->stats = 0;
}

static inline void taskstats_tgid_free(struct signal_struct *sig)
{
 if (sig->stats)
  kmem_cache_free(taskstats_cache, sig->stats);
}

extern void taskstats_exit(struct task_struct *, int group_dead);
extern void taskstats_init_early(void);
extern int delayacct_on;
extern struct kmem_cache *delayacct_cache;
extern void delayacct_init(void);
extern void __delayacct_tsk_init(struct task_struct *);
extern void __delayacct_tsk_exit(struct task_struct *);
extern void __delayacct_blkio_start(void);
extern void __delayacct_blkio_end(void);
extern int __delayacct_add_tsk(struct taskstats *, struct task_struct *);
extern __u64 __delayacct_blkio_ticks(struct task_struct *);
extern void __delayacct_freepages_start(void);
extern void __delayacct_freepages_end(void);

static inline int delayacct_is_task_waiting_on_io(struct task_struct *p)
{
 if (p->delays)
  return (p->delays->flags & 0x00000002);
 else
  return 0;
}

static inline void delayacct_set_flag(int flag)
{
 if (get_current()->delays)
  get_current()->delays->flags |= flag;
}

static inline void delayacct_clear_flag(int flag)
{
 if (get_current()->delays)
  get_current()->delays->flags &= ~flag;
}

static inline void delayacct_tsk_init(struct task_struct *tsk)
{

 tsk->delays = 0;
 if (delayacct_on)
  __delayacct_tsk_init(tsk);
}




static inline void delayacct_tsk_free(struct task_struct *tsk)
{
 if (tsk->delays)
  kmem_cache_free(delayacct_cache, tsk->delays);
 tsk->delays = 0;
}

static inline void delayacct_blkio_start(void)
{
 delayacct_set_flag(0x00000002);
 if (get_current()->delays)
  __delayacct_blkio_start();
}

static inline void delayacct_blkio_end(void)
{
 if (get_current()->delays)
  __delayacct_blkio_end();
 delayacct_clear_flag(0x00000002);
}

static inline int delayacct_add_tsk(struct taskstats *d,
     struct task_struct *tsk)
{
 if (!delayacct_on || !tsk->delays)
  return 0;
 return __delayacct_add_tsk(d, tsk);
}

static inline __u64 delayacct_blkio_ticks(struct task_struct *tsk)
{
 if (tsk->delays)
  return __delayacct_blkio_ticks(tsk);
 return 0;
}

static inline void delayacct_freepages_start(void)
{
 if (get_current()->delays)
  __delayacct_freepages_start();
}

static inline void delayacct_freepages_end(void)
{
 if (get_current()->delays)
  __delayacct_freepages_end();
}
__u64 __delayacct_blkio_ticks(struct task_struct *tsk)
{
 __u64 ret;
 unsigned long flags;

 ;
 ret = nsec_to_clock_t(tsk->delays->blkio_delay +
    tsk->delays->swapin_delay);
 ;
 return ret;
}