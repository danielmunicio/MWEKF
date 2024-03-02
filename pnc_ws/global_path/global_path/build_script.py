from global_opt_compiled import CompiledGlobalOpt
from global_opt_settings import GlobalOptSettings

def build_solver():
    g = CompiledGlobalOpt(**GlobalOptSettings)
    g.construct_solver(generate_c=True, compile_c=True, gcc_opt_flag='-O3')

if __name__ == '__main__':
    build_solver()