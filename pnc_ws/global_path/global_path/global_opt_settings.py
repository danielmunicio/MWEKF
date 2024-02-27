class settings:
    N = 100
    solver = 'ipopt'
    car_params = {'l_r': 1.4987, 'l_f':1.5213, 'm': 1.}
    bbox={'l': 2, 'w': 1}
    DF_MAX  =  0.5
    ACC_MIN = -3.0
    ACC_MAX_FN = 2.0
    DF_DOT_MAX =  0.5
    V_MIN = 0.0
    V_MAX = 25.0
    FRIC_MAX = 12.0

    @classmethod
    def settings():
        return 