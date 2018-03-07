import vispy.scene
from vispy.scene import visuals
from vispy import scene
import numpy as np
from vispy import app, gloo 
from vispy import visuals as viz
from vispy.visuals.transforms import (STTransform, LogTransform,
                                      MatrixTransform, PolarTransform)

# vertex positions of data to draw
N = 2
pos = np.zeros((N, 2), dtype=np.float32)
pos[:, 0] = np.linspace(0, 350, N)
# pos[:, 1] = np.random.normal(size=N, scale=50, loc=0)
pos[:, 1] = np.linspace(0, 350, N)

# One array of colors
color = np.ones((N, 4), dtype=np.float32)
color[:, 0] = np.linspace(0, 1, N)
color[:, 1] = color[::-1, 0]

vertex_shader = """
void main()
{
    vec4 visual_pos = vec4($position, 1);
    vec4 doc_pos = $visual_to_doc(visual_pos);

    gl_Position = $doc_to_render(doc_pos);
}
"""

fragment_shader = """
void main() {
  gl_FragColor = $color;
}
"""
RED = (1.0, 0.0, 0.0, 1.0)
GREEN = (0.0, 1.0, 0.0, 1.0)
BLUE = (0.0, 0.0, 1.0, 1.0)

class Plot3DVisual(viz.Visual):
    """ template """

    def __init__(self, x, y, z,_col):
        """ plot 3D """
        viz.Visual.__init__(self, vertex_shader, fragment_shader)

        # build Vertices buffer
        data = np.c_[x, y, z]
        v = gloo.VertexBuffer(data.astype(np.float32))

        # bind data
        self.shared_program.vert['position'] = v
        # self.shared_program.frag['color'] = (1.0, 0.0, 0.0, 1.0)
        self.shared_program.frag['color'] = _col

        # config
        self.set_gl_state('opaque', clear_color=(1, 1, 1, 1))
        self._draw_mode = 'line_strip'

    def _prepare_transforms(self, view):
        """ This method is called when the user or the scenegraph has assigned
        new transforms to this visual """
        # Note we use the "additive" GL blending settings so that we do not
        # have to sort the mesh triangles back-to-front before each draw.
        tr = view.transforms
        view_vert = view.view_program.vert
        view_vert['visual_to_doc'] = tr.get_transform('visual', 'document')
        view_vert['doc_to_render'] = tr.get_transform('document', 'render')

class DisplayCloudVispy:
    def __init__(self):
        self.canvas = vispy.scene.SceneCanvas(keys='interactive', show=True)
        self.view = self.canvas.central_widget.add_view()
        self.scatter = visuals.Markers()
        self.Plot3D = scene.visuals.create_visual_node(Plot3DVisual)
       

    def draw_axis(self,centroid,basis):
        N = 2
        # basis = [[1,  0,  0],
        #         [ 0, 1, 0],
        #         [0,  0, 1]]
        _col = [ RED, GREEN, BLUE]
        k=0.1
        for i in range(3):
            # print("basis",basis[i])
            x = np.linspace(centroid[0],(centroid[0]+k*basis[i][0]), N)
            y = np.linspace(centroid[1],(centroid[1]+k*basis[i][1]), N)
            z = np.linspace(centroid[2],(centroid[2]+k*basis[i][2]), N)
            # print("x",x)
            p1 = self.Plot3D(x, y, z,_col[i],parent=self.view.scene)
        
    def draw_cloud(self,data,centroid,basis): 
        self.draw_axis(centroid,basis)
        self.scatter.set_data(data, edge_color=None, face_color=(1, 1, 1, .5), size=5)
        self.view.add(self.scatter)
        self.view.camera = 'turntable'
        axis = visuals.XYZAxis(parent=self.view.scene)
    
    def show(self): 
        vispy.app.run()

    def quit(self):
        vispy.app.quit()     