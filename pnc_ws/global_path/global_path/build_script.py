from global_opt_compiled import CompiledGlobalOpt
from global_opt_settings import GlobalOptSettings

if __name__ == '__main__':
    g = CompiledGlobalOpt(**GlobalOptSettings)
    g.construct_solver(generate_c=True, compile_c=True)