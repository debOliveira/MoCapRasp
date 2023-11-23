import numpy as np
import plotly.graph_objects as go

class Frame: 
    def __init__(self, R=np.eye(3), t=np.zeros((3, 1))):
        self.R = R
        self.t = t

class ArenaViewer: 
    def __init__(self, title, arenaSize=0, plotSize=(900, 700), reference=True, graphical=False):
        self.title = title
        self.graphical = graphical # Toggle to activate graphical mode
        self.arenaSize = arenaSize # Change viewable graph dimensions 
        self.plotSize = plotSize

        # Create Figure 
        self.figure = go.Figure(
            layout=go.Layout(
                width=self.plotSize[0],
                height=self.plotSize[1], 
                title=go.layout.Title(text=self.title)
            )
        )

        # Set up layout enviroment
        self.figure.update_layout(
            scene_aspectmode='cube',
            scene = dict(
                xaxis_title='x'*graphical,
                yaxis_title='y'*graphical, 
                zaxis_title='z'*graphical, 
                xaxis=dict(
                    range=[-arenaSize,arenaSize] if reference else None,
                    showbackground=graphical,
                    showticklabels=graphical,
                    showaxeslabels=graphical,
                    showgrid=True,
                    showspikes=graphical
                    ),
                yaxis=dict(
                    range=[-arenaSize,arenaSize] if reference else None,
                    showline=False,
                    showbackground=graphical,
                    showticklabels=graphical,
                    showaxeslabels=graphical,
                    showgrid=True,
                    showspikes=graphical
                    ), 
                zaxis=dict(
                    range=[-1e-9,2*arenaSize-1e-9] if reference else None, # Room for negative z markers in the Ground Wand
                    showbackground=reference,
                    showticklabels=graphical,
                    showaxeslabels=graphical,
                    showgrid=graphical,
                    showspikes=graphical
                    )
            )
        )

        # Change camera settings
        self.figure.update_layout(
            scene=dict(
                camera=dict(
                    projection=dict(
                        type='orthographic'
                    )
                )
            )
        )

    # Reference methods
    def add_origin(self):
        self.figure.add_trace(
            go.Scatter3d(
                x=[0],
                y=[0],
                z=[0],
                mode='markers',
                marker=dict(
                    size=2,
                    opacity=0.75,
                    color='black'
                ),
                name='Origin',
                legendgroup='References',
                legendgrouptitle_text='Refereces',
                hoverinfo='skip',
                showlegend=True
            )
        )

    def add_boundary(self, points, name, color=None):
        points = np.hstack([points, points[:,[0]]]) # Repeat first column in the last column

        self.figure.add_trace(
            go.Scatter3d(
                x=points[0],
                y=points[1],
                z=points[2],
                mode='lines',
                line=dict(
                    width=2,
                    color=color,
                    dash='dash'
                    ),
                opacity=0.5,
                name=name,
                legendgroup='References',
                legendgrouptitle_text='References',
                hoverinfo='skip',
                showlegend=True
            )
        )

    def add_plane(self, vertices, name, color=None):
        self.figure.add_trace(
            go.Mesh3d(
                x=vertices[0],
                y=vertices[1],
                z=vertices[2],
                opacity=0.5,
                color=color,
                flatshading=True,
                name=name,
                legendgroup='References',
                legendgrouptitle_text='References',
                hoverinfo='skip',
                showlegend=True
            )
        )

    # Arena elements methods
    def add_frame(self, frame, name, axis_size=1, color=None):

        # Set default colors
        axis_name_list = ['x', 'y', 'z']
        axis_color_list = ['red', 'green', 'blue']

        self.figure.add_trace(
            go.Scatter3d(
                x=frame.t[0],
                y=frame.t[1],
                z=frame.t[2],
                mode='markers',
                marker=dict(
                    size=4,
                    opacity=0.80,
                    color=color
                ),
                name=name,
                legendgroup='Frames',
                legendgrouptitle_text='Frames',
                showlegend=True
            )
        )

        for axis, axis_color in enumerate(axis_color_list):

            arrow = np.hstack((frame.t, frame.t + frame.R[:,axis].reshape(-1,1) * axis_size)) # Arrow of an axis

            self.figure.add_trace(
                go.Scatter3d(
                    x=arrow[0], 
                    y=arrow[1],
                    z=arrow[2], 
                    mode='lines',
                    line=dict(
                        width=2,
                        color=axis_color
                        ),
                    showlegend=False,
                    name=axis_name_list[axis]+name,
                    hoverinfo = None if self.graphical else 'skip'
                )
            )

    def add_path(self, points, name, color=None):
        self.figure.add_trace(
            go.Scatter3d(
                x=points[0],
                y=points[1],
                z=points[2],
                mode='markers',
                marker=dict(
                    size=2,
                    opacity=0.01,
                    color=color
                ),
                name=name,
                legendgroup='Paths',
                legendgrouptitle_text='Paths',
                showlegend=True
            )
        )

    def add_markers(self, points, name, color=None):
        self.figure.add_trace(
            go.Scatter3d(
                x=points[0],
                y=points[1],
                z=points[2],
                mode='markers',
                marker=dict(
                    size=3,
                    opacity=1,
                    color=color
                ),
                name=name,
                legendgroup='Markers',
                legendgrouptitle_text='Markers',
                showlegend=True
            )
        )