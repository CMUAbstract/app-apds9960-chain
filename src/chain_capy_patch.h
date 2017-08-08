#ifndef CHAIN_CAPY_PATCH_
#define CHAIN_CAPY_PATCH
#include <libcapybara/reconfig.h>

typedef capybara_cfg_spec_t task_cfg_spec_t; 
typedef capybara_cfg_t task_opcfg_t ;
typedef capybara_cfg_t task_precfg_t ;

typedef struct capy_pwrcfg_info_{
    task_cfg_spec_t spec_cfg; 
    task_opcfg_t opcfg; 
    task_precfg_t precfg; 
} capy_pwrcfg_info_t, *capy_pwrcfg_info_ptr; 

// To characterize the power config of a function we have three fields: 
    // cfg_spec indicates if a power config is specified for the task
    // opcfg is the object which holds the power configuration if cfg_spec > 0 
    // precfg is the power configuration we're precharging to-- only important
    // for preburst tasks
    // TODO automatically set precfg to satisfy the burst requirements of the
    // most power hungry burst task that follows a preburst, and throw an error
    // if a preburst task is followed by a task with a greater energy usage than
    // the precfg level 

extern  capy_pwrcfg_info_ptr * volatile task_pwr_info; 

#define PWR_SYM_NAME(func) _pwrcfg_ ## func
#define PWR_SYM_REF(func) &PWR_SYM_NAME(func)


#define TASK_BASIC(idx, func) \
    void func(); \
    __nv task_t TASK_SYM_NAME(func) = { func, (1UL << idx), idx,{0},0,0, #func }; \
    __nv capy_pwrcfg_info_t PWR_SYM_NAME(func)  = { DEFAULT, NULL, NULL}; \
    task_pwr_info[idx] = PWR_SYM_REF(func) ; 

#define TASK_DEFBUR(idx, func, spec_cfg) \
    void func(); \
    __nv task_t TASK_SYM_NAME(func) = { func, (1UL << idx), idx,{0},0,0, #func }; \
    __nv capy_pwrcfg_info_t PWR_SYM_NAME(func) = { spec_cfg, NULL, NULL}; \
    task_pwr_info[idx] = PWR_SYM_REF(func) ; 

#define TASK_CONFIGD(idx, func, spec_cfg, pwr_level) \
    void func(); \
    __nv task_t TASK_SYM_NAME(func) = { func, (1UL << idx), idx,{0},0,0, #func }; \
    __nv capy_pwrcfg_info_t func ## _pwr = { spec_cfg, pwr_levels + pwr_level, NULL}; \
    task_pwr_info[idx] = &(func ## _pwr ) ; 

#define TASK_PREBURST(idx, func, spec_cfg, burst_level, op_pwr_level) \
    void func(); \
    __nv task_t TASK_SYM_NAME(func) = { func, (1UL << idx), idx,{0},0,0, #func }; \
    __nv capy_pwrcfg_info_t func ## _pwr = { spec_cfg, pwr_levels + op_pwr_level, \
                                   pwr_levels + burst_level}; \
    task_pwr_info[idx] = &(func ## _pwr ) ; 

#define GET_TASK_MACRO(_1,_2,_3,_4,_5,NAME,...) NAME

#define SET_TASK(...) \
    GET_TASK_MACRO(__VA_ARGS__,TASK_PREBURST,TASK_CONFIGD,\
        TASK_DEFBUR, TASK_BASIC )(__VA_ARGS__) \

#undef TASK
#define TASK(...) \
    SET_TASK(__VA_ARGS__) \

#undef ENTRY_TASK
#undef _entry_task 
#undef _task__entry_task

#define ENTRY_TASK(task) \
    TASK(0, _entry_task) \
    void _entry_task() { TRANSITION_TO(task); } \


#endif //CHAIN_CAPY_PATCH
