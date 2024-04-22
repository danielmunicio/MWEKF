#!python
from .local_opt_compiled import CompiledLocalOpt
from all_settings.all_settings import LocalOptSettings

def build_solver():
    g = CompiledLocalOpt(**LocalOptSettings)
    g.construct_solver(generate_c=True, compile_c=True, gcc_opt_flag='-O3')

if __name__ == '__main__':
    build_solver()
