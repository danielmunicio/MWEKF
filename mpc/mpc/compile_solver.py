#!python
from .mpc_controller import MPCPathFollower
from all_settings.all_settings import MPCSettings as settings

def build_solver():
    MPCPathFollower(**settings).construct_solver(generate_c=True, compile_c=True, gcc_opt_flag='-Ofast')

if __name__ == '__main__':
    build_solver()