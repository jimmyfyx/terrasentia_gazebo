import numpy as np
import casadi as ca
from time import time

class MPC_CONTROLLER:
    def __init__(self, params):

        self.p = {}
        # Set params
        self.p['dt']         = params['dt']          # time between steps in seconds
        self.p['N']          = params['N']           # number of look ahead steps
        self.p['Lbase']      = params['Lbase']       # wheel base of the robot
        self.p['v_max']      = params['v_max']       # maximum motor speed output
        self.p['v_lin_max']  = params['v_lin_max']   # 
        self.p['v_ang_max']  = params['v_ang_max']   #
        self.p['verbose']      = params['verbose']
        self.p['n_states']   = params['n_states']    #
        self.p['n_controls'] = params['n_controls']
        self.p['mu']         = params['mu']          # linear skid coefficient TODO test from mhe
        self.p['nu']         = params['nu']          # angular skid coefficient
        self.p['eps']        = params['eps']
        self.p['alpha']      = params['alpha']
        self.p['gain_ctrack_error_x'] = params['gain_ctrack_error_x']
        self.p['gain_ctrack_error_y'] = params['gain_ctrack_error_y']
        self.p['gain_ctrack_error_theta'] = params['gain_ctrack_error_theta']
        self.p['terminal_cost_multiplier_x'] = params['terminal_cost_multiplier_x']
        self.p['terminal_cost_multiplier_y'] = params['terminal_cost_multiplier_y']
        self.p['terminal_cost_multiplier_theta'] = params['terminal_cost_multiplier_theta']
        self.p['gain_control_effort_linear'] = params['gain_control_effort_linear']
        self.p['gain_control_effort_angular'] = params['gain_control_effort_angular']
        self.use_delay = params['use_delay']
        
        self.setMatrices()

        self.printParams()
        self.x_start = 0
        self.u_start = self.p['n_states']*(self.p['N']+1)

        lbx = -ca.DM.inf(( self.p['n_states']*(self.p['N']+1) + self.p['n_controls']*self.p['N'], 1))
        ubx =  ca.DM.inf(( self.p['n_states']*(self.p['N']+1) + self.p['n_controls']*self.p['N'], 1))

        lbx[self.u_start : : 2] = -self.p['v_lin_max']      # v lower bound for v
        lbx[self.u_start+1 : : 2] = -self.p['v_ang_max']    # omega lower bound for
        ubx[self.u_start : : 2] = self.p['v_lin_max']       # v upper bound for v
        ubx[self.u_start+1 : : 2] = self.p['v_ang_max']     # omega upper bound for
        
        print('lbx:', lbx)

        lbg = ca.DM.zeros(( self.p['n_states']*(self.p['N']+1) + 2*self.p['N'], 1))
        ubg = ca.DM.zeros(( self.p['n_states']*(self.p['N']+1) + 2*self.p['N'], 1))

        lbg[self.p['n_states']*(self.p['N']+1) : self.p['n_states']*(self.p['N']+1) + self.p['n_controls']*self.p['N']] = -self.p['v_max'] 
        ubg[self.p['n_states']*(self.p['N']+1) : self.p['n_states']*(self.p['N']+1) + self.p['n_controls']*self.p['N']] =  self.p['v_max'] 

        print('lbg:', lbg)

        self.args = {'lbg': lbg,  # constraints lower bound
                     'ubg': ubg,  # constraints upper bound
                     'lbx': lbx,
                     'ubx': ubx}

        self.set_x_dot()
        self.set_mpc()

    def updateParams(self, params):
        for param in params:
            if param in list(self.p):
                if self.p[param] != params[param]:
                    print('mpc updating parameter [' + str(param) + '] from ' + str(self.p[param]) + ' to ' + str(params[param]))
                    self.p[param] = params[param]
        self.setMatrices()
        return True

    def setMatrices(self):
        # state weights matrix (Q_X, Q_Y, Q_THETA) ctrack
        if self.use_delay:
            self.Q  = ca.diagcat(self.p['gain_ctrack_error_x'], self.p['gain_ctrack_error_y'], self.p['gain_ctrack_error_theta'], 0)
            self.Qf = ca.diagcat(self.p['terminal_cost_multiplier_x'], self.p['terminal_cost_multiplier_y'], self.p['terminal_cost_multiplier_theta'], 0) #terminal_cost_multiplier 
        else:
            self.Q  = ca.diagcat(self.p['gain_ctrack_error_x'], self.p['gain_ctrack_error_y'], self.p['gain_ctrack_error_theta'])
            self.Qf = ca.diagcat(self.p['terminal_cost_multiplier_x'], self.p['terminal_cost_multiplier_y'], self.p['terminal_cost_multiplier_theta']) #terminal_cost_multiplier 
        # controls weights matrix
        self.R  = ca.diagcat(self.p['gain_control_effort_linear'], self.p['gain_control_effort_angular']) #gain_control_effort linear, angular

    def printParams(self):
        print('mpc params:' + \
            '\n\tdt: ' + str(self.p['dt']) + \
            '\n\tN: ' + str(self.p['N']) + \
            '\n\tLbase: ' + str(self.p['Lbase']) + \
            '\n\tv_max: ' + str(self.p['v_max']) + \
            '\n\tv_lin_max: ' + str(self.p['v_lin_max']) + \
            '\n\tv_ang_max: ' + str(self.p['v_ang_max']) + \
            '\n\tverbose: ' + str(self.p['verbose']) + \
            '\n\tn_states: ' + str(self.p['n_states']) + \
            '\n\tn_controls: ' + str(self.p['n_controls']) + \
            '\n\tmu: ' + str(self.p['mu']) + \
            '\n\tnu: ' + str(self.p['nu']) + \
            '\n\teps: ' + str(self.p['eps']) + \
            '\n\tgain_ctrack_error_x: ' + str(self.p['gain_ctrack_error_x']) + \
            '\n\tgain_ctrack_error_y: ' + str(self.p['gain_ctrack_error_y']) + \
            '\n\tgain_ctrack_error_theta: ' + str(self.p['gain_ctrack_error_theta']) + \
            '\n\tterminal_cost_multiplier_x: ' + str(self.p['terminal_cost_multiplier_x']) + \
            '\n\tterminal_cost_multiplier_y: ' + str(self.p['terminal_cost_multiplier_y']) + \
            '\n\tterminal_cost_multiplier_theta: ' + str(self.p['terminal_cost_multiplier_theta']) + \
            '\n\tgain_control_effort_linear: ' + str(self.p['gain_control_effort_linear']) + \
            '\n\tgain_control_effort_angular: ' + str(self.p['gain_control_effort_angular'])
            )
    def set_x_dot(self):
        '''
        Lets define the integrator function
        '''
        # state symbolic variables
        states = ca.SX.sym('x', self.p['n_states'])
        # control symbolic variables
        controls = ca.SX.sym('u', self.p['n_controls'])
        # reference speed
        vref = ca.SX.sym('vref')

        mu = self.p['mu']    #ca.SX.sym('mu')
        nu = self.p['nu']    #0.4
        if self.use_delay:
            RHS = ca.vertcat(
                (controls[0]+vref) * mu*ca.cos(states[2]),
                (controls[0]+vref) * mu*ca.sin(states[2]),
                nu * states[3],
                self.p['alpha']*(controls[1] - states[3]))
        else:
            RHS = ca.vertcat(
                (controls[0]+vref) * mu*ca.cos(states[2]),
                (controls[0]+vref) * mu*ca.sin(states[2]),
                controls[1] * nu)

        # maps controls from [v, omega].T to [vx, vy, omega].T
        self.x_dot = ca.Function('f', [states, controls, vref], [RHS])

    def set_mpc(self):
        '''
        Now we define the optimization horizon
        '''
        # matrix containing all states over all time steps +1 (each column is a state vector)
        X = ca.SX.sym('X', self.p['n_states'], self.p['N']+1)
        # matrix containing all control actions over all time steps (each column is an action vector)
        U = ca.SX.sym('U', self.p['n_controls'], self.p['N'])
        # column vector for storing initial state and target state
        P = ca.SX.sym('P', self.p['n_states'] + (self.p['N']+1) * self.p['n_states'] + self.p['N'])

        # Initialize cost function
        cost_fn = 0
        # Constraint the initial states
        g = X[:, 0] - P[:self.p['n_states']]

        ref = ca.reshape(P[self.p['n_states'] : self.p['n_states'] + self.p['n_states']*(self.p['N']+1)], self.p['n_states'], self.p['N']+1)
        vref = ca.reshape(P[-self.p['N']:], 1, self.p['N'])

        # Runge-Kutta integration
        for k in range(self.p['N']):
            st = X[:, k]
            con = U[:, k]

            error = st - ref[:,k]
            #error[2] = ca.mod(error[2] + ca.pi, 2*ca.pi) - ca.pi
            error[2] = (1 - ca.cos(error[2]))**2
            cost_fn += error.T @ self.Q @ error
            cost_fn += con.T @ self.R @ con

            st_next = X[:, k+1]
            k1 = self.x_dot(st, con, vref[0,k])

            st2 = st + self.p['dt']/2*k1
            k2 = self.x_dot(st2, con, vref[0,k])

            st3 = st + self.p['dt']/2*k2
            k3 = self.x_dot(st3, con, vref[0,k])

            st4 = st + self.p['dt'] * k3
            k4 = self.x_dot(st4, con, vref[0,k])

            st_next_RK4 = st + (self.p['dt'] / 6) * (k1 + 2*k2 + 2*k3 + k4)
            g = ca.vertcat(g, st_next - st_next_RK4)

        # Final cost
        st = X[:, self.p['N']]
        cost_fn += (st - ref[:,self.p['N']]).T @ self.Qf @ (st - ref[:,self.p['N']])

        # Constraints for max motor speed
        g = ca.vertcat(g, U[0,:].T + vref[0,:].T + 0.5*self.p['Lbase']*U[1,:].T)
        g = ca.vertcat(g, U[0,:].T + vref[0,:].T - 0.5*self.p['Lbase']*U[1,:].T)

        # Set the optmization variables
        OPT_variables = ca.vertcat(X.reshape((-1, 1)), U.reshape((-1, 1)))

        nlp_prob = {'f': cost_fn,
                    'x': OPT_variables,
                    'g': g,
                    'p': P}

        opts = {'ipopt': {'max_iter': 50,
                          'print_level': 0,
                          'max_cpu_time': 0.5},
                'print_time': 0,
                'jit': True,
                'compiler': 'shell',
                'jit_options': {'compiler': 'gcc', 'flags': ['-O3']}}

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    def solve_mpc(self, mpc_reference, omega=0):
        # Set initial states
        if self.use_delay:
                state_init = ca.DM([0.0, 0.0, 0.0, omega])
        else:
                state_init = ca.DM([0.0, 0.0, 0.0])
		

        # Define target states
        ref = []
        for i in range(self.p['N']+1):
            if self.use_delay:
                ref.extend([mpc_reference['x'][i], mpc_reference['y'][i], mpc_reference['theta'][i], 0])
            else:
                ref.extend([mpc_reference['x'][i], mpc_reference['y'][i], mpc_reference['theta'][i]])
        state_target = ca.DM(ref)

        vref = ca.DM(mpc_reference['speed'][:-1])
        # if vref[0] < 0 :
        #     state_init = ca.DM([0.0, 0.0, mpc_reference['theta'][0]])
        # print('mpc reference theta ' + str(mpc_reference['theta'][0]))
        # Print verbose messages
        if self.p['verbose']:
        # if vref[0] < 0:
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
            print('state_target:', state_target)
            print('mpc_reference[x]:', mpc_reference['x'])
            print('mpc_reference[y]:', mpc_reference['y'])
            print('mpc_reference[theta]:', mpc_reference['theta'])
            print('reference speed: ', vref)
            print('state init: ', state_init)

        # initial states and control replicated for the whole horizon
        X0 = ca.repmat(state_init, 1, self.p['N']+1) 
        u0 = ca.DM.zeros((self.p['n_controls'], self.p['N']))

        # optimization variable current state
        x0 = ca.vertcat(ca.reshape(X0, self.p['n_states']*(self.p['N']+1), 1),
                        ca.reshape(u0, self.p['n_controls']*self.p['N'], 1))

        p = ca.vertcat(state_init,      # initial states
                       state_target,    # target states
                       vref)            # reference speed param

        sol = self.solver(x0=x0,
                          lbx=self.args['lbx'],
                          ubx=self.args['ubx'],
                          lbg=self.args['lbg'],
                          ubg=self.args['ubg'],
                          p=p)

        u = ca.reshape(sol['x'][self.u_start:], self.p['n_controls'], self.p['N'])
        pred_vals = ca.reshape(sol['x'][:self.p['n_states']*(self.p['N']+1)], self.p['n_states'], self.p['N']+1)

        ss_error = np.linalg.norm([state_init[0] - mpc_reference['x'][0],
                                   state_init[1] - mpc_reference['x'][1]])

        # sol['f'] cost

        # Adjust the output linear velocity
        u[0,:] += vref.T

        # print('pred_vals: ', pred_vals)
        return u, pred_vals, ss_error
