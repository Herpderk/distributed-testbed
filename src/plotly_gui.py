import dash
from dash import dcc
from dash import html
from dash.dependencies import Output, Input
import plotly
import plotly.graph_objs as go
from multiprocess import Manager
import numpy as np
import random


class Plot_Settings:


    def __init__(self):
        # dimensions of 3d space in mm
        self.x_dim = 3
        self.y_dim = 3
        self.z_dim = 3
        # colors and number of robots, index is the id
        tracker_colors = np.array([
            'rP', 'gP', 'bP'
        ])
        self.num_robots = len(tracker_colors)
        # labels for each robot
        self.labels = np.empty(self.num_robots, dtype=str)
        for i in range(self.num_robots):
            self.labels[i] = ('robot' + str(i))


    def update_coords(self, id, position):
        x_coords[id] = position[0]
        y_coords[id] = position[1]
        z_coords[id] = position[2]


# stuff to set up Dash app
plot = Plot_Settings()
'''
manager = Manager()
x_coords = manager.list(range(plot.num_robots))
y_coords = manager.list(range(plot.num_robots))
z_coords = manager.list(range(plot.num_robots))
'''
x_coords = np.zeros(plot.num_robots)
y_coords = np.zeros(plot.num_robots)
z_coords = np.zeros(plot.num_robots)


app = dash.Dash(__name__)
app.layout = html.Div([
	dcc.Graph(id='live-graph', animate=False), dcc.Interval(
		id ='graph-update',
		interval = 0.1*1000
		),
	]
)


@app.callback(
	Output('live-graph', 'figure'), 
	[Input('graph-update', 'n_intervals')]
)


def update_graph(input_data):
	'''
    print('Xs: ' + str(x_coords))
    print('Ys: ' + str(y_coords))
    print('Zs: ' + str(z_coords))
    print()
    '''
	data = go.Scatter(
		x = x_coords,
		y = y_coords,
		name = 'Scatter',
		mode ='markers'
	)
	return {'data': [data], 'layout': 
		go.Layout(
			xaxis = dict(range=[0, plot.x_dim]),
			yaxis = dict(range=[0, plot.y_dim]),
			width = 700, height=700)
	}


if __name__ == '__main__':
    app.run_server(port=8080, debug=True)
    while True:
    	for id in range(n):
        	plot.update_coords(id, plot.x_dim*np.random.rand(n))
        	
