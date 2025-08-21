import numpy as np
import moderngl

from astroviz.common._utils import (
    RotX, RotY, RotZ, RotIdentity, VectZero, VectOne, MatModel, MatNormal
)

class ShapeTriangle:
    """
    Triángulo básico para tests.
    - Shader con uniforms: camera (mat4), mat_model (mat4), color (vec4).
    - Vértices en espacio local; se transforman con mat_model y luego con camera.
    - 'camera' debe ser una mat4 column-major (si pasas numpy 4x4, se transpone).
      Con PyGLM usa: camera.to_bytes()
    """

    def __init__(self, ctx: moderngl.Context, color=(1.0, 1.0, 1.0, 1.0), vertices=None):
        self.ctx = ctx
        self.color = np.array(color, dtype=np.float32)

        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 camera;
                uniform mat4 mat_model;
                layout (location = 0) in vec3 in_vertex;
                void main() {
                    gl_Position = camera * mat_model * vec4(in_vertex, 1.0);
                }
            """,
            fragment_shader="""
                #version 330 core
                uniform vec4 color;
                layout (location = 0) out vec4 out_color;
                void main() {
                    out_color = color;
                }
            """
        )

        # Uniform handles
        self.u_camera   = self.prog["camera"]
        self.u_model    = self.prog["mat_model"]
        self.u_color    = self.prog["color"]

        # Vértices por defecto (igual a tu ejemplo)
        if vertices is None:
            vertices = np.array([
                 0.0,  0.4, 0.0,
                -0.4, -0.3, 0.0,
                 0.4, -0.3, 0.0,
            ], dtype=np.float32)
        else:
            vertices = np.asarray(vertices, dtype=np.float32).reshape(-1, 3)

        self.vbo = ctx.buffer(vertices.tobytes())
        self.vao = ctx.vertex_array(self.prog, [(self.vbo, "3f", "in_vertex")])

    # --------- API de conveniencia ----------
    def set_color(self, rgba):
        """Cambia el color (tuple/list de 4 floats)."""
        self.color = np.array(rgba, dtype=np.float32)

    def update_vertices(self, vertices):
        """Reemplaza los vértices del triángulo (Nx3)."""
        arr = np.asarray(vertices, dtype=np.float32).reshape(-1, 3)
        # orphan para evitar stalling en la GPU cuando el tamaño cambia
        self.vbo.orphan(len(arr) * 3 * 4)
        self.vbo.write(arr.tobytes())

    # --------- Render ----------
    def render(self, camera_mat, pos=VectZero(), rot=RotIdentity(), scale=1.0):
        """
        Dibuja el triángulo.
        - camera_mat: puede ser glm.mat4 (usa .to_bytes()) o numpy (4x4, se transpone).
        - pos/rot/scale: se pasan a MatModel del utils.
        """
        # mat_model en row-major → bytes contiguos
        mat_model = MatModel(pos, rot, scale * VectOne()).astype("f4")
        self.u_model.write(mat_model.tobytes())

        # camera en column-major:
        # - si es glm.mat4 → to_bytes() ya está column-major
        # - si es numpy 4x4, asumimos row-major → transponemos
        if hasattr(camera_mat, "to_bytes"):
            self.u_camera.write(camera_mat.to_bytes())
        else:
            cam = np.asarray(camera_mat, dtype=np.float32)
            assert cam.shape == (4, 4), "camera_mat debe ser 4x4"
            self.u_camera.write(cam.T.tobytes())

        self.u_color.write(self.color.tobytes())
        self.vao.render(moderngl.TRIANGLES)

    def release(self):
        """Libera recursos GPU (llamar al cerrar)."""
        try:
            if self.vao: self.vao.release()
            if self.vbo: self.vbo.release()
            if self.prog: self.prog.release()
        except Exception:
            pass

class ShapeTrajectory():
    def __init__(self, ctx, points, color=(1.0, 0.0, 0.0, 1.0)):
        self.ctx = ctx
        self.color = color
        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 mat_proj;
                in vec3 pos;
                void main() {
                    gl_Position = mat_proj * vec4(pos, 1.0);
                }
            """,
            fragment_shader="""
                #version 330 core
                uniform vec4 color;
                out vec4 fragment_color;
                void main() {
                    fragment_color = color;
                }
            """,
        )
        self.uniform_mat_proj = self.prog["mat_proj"]
        self.uniform_color = self.prog["color"]

        self.update_points(points)

    def update_points(self, points):
        if len(points) < 2:
            self.vbo_lines = None
            self.vao_lines = None
            self.vbo_arrows = None
            self.vao_arrows = None
            return

        # Generate line segments
        lines = []
        arrow_triangles = []
        arrow_size = 0.02
        arrow_width = 0.01

        for i in range(len(points) - 1):
            # Line segment
            start = np.array(points[i])
            end = np.array(points[i + 1])
            lines.extend([start, end])

            # Arrowhead (triangle)
            direction = end - start
            direction = direction / np.linalg.norm(direction)

            perp = np.cross(direction, [0, 0, 1])
            if np.linalg.norm(perp) < 1e-6:
                perp = np.cross(direction, [0, 1, 0])
            perp = perp / np.linalg.norm(perp) * arrow_width

            # Define the triangle points
            tip = end
            left = end - direction * arrow_size + perp
            right = end - direction * arrow_size - perp
            arrow_triangles.extend([tip, left, right])

        # Create VBOs and VAOs for lines and arrowheads
        self.vbo_lines = self.ctx.buffer(np.array(lines).astype(np.float32).tobytes())
        self.vao_lines = self.ctx.vertex_array(self.prog, [(self.vbo_lines, "3f4", "pos")])

        self.vbo_arrows = self.ctx.buffer(np.array(arrow_triangles).astype(np.float32).tobytes())
        self.vao_arrows = self.ctx.vertex_array(self.prog, [(self.vbo_arrows, "3f4", "pos")])

    def render(self, cam):
        if self.vao_lines:
            self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
            self.uniform_color.value = self.color
            self.vao_lines.render(mode=moderngl.LINES)

        if self.vao_arrows:
            self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
            self.uniform_color.value = self.color
            self.vao_arrows.render(mode=moderngl.TRIANGLES)

    def release(self):
        self.prog.release()
        if self.vbo_lines:
            self.vbo_lines.release()
        if self.vao_lines:
            self.vao_lines.release()
        if self.vbo_arrows:
            self.vbo_arrows.release()
        if self.vao_arrows:
            self.vao_arrows.release()

class ShapeGrid():
    def __init__(self, ctx, length=10.0, segments=10):
        self.ctx = ctx
        #Program shaders
        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 mat_proj;
                in vec3 pos;
                void main() {
                    gl_Position = mat_proj*vec4(pos, 1.0);
                }
            """,
            fragment_shader="""
                #version 330 core
                out vec4 fragment_color;
                void main() {
                    fragment_color = vec4(1.0, 1.0, 1.0, 0.1);
                }
            """,
        )
        self.uniform_mat_proj = self.prog["mat_proj"]
        #Generate vertices
        list_vertex = []
        for x in np.linspace(-length, length, 2*segments+1):
            list_vertex += [x, length, -1.0]
            list_vertex += [x, -length, -1.0]
        for y in np.linspace(-length, length, 2*segments+1):
            list_vertex += [length, y, -1.0]
            list_vertex += [-length, y, -1.0]
        #Create buffers
        self.vbo = ctx.buffer(np.array(list_vertex).astype(np.float32).tobytes())
        self.vao = ctx.vertex_array(self.prog, [(self.vbo, "3f4", "pos")])
    def render(self, cam):
        self.ctx.line_width = 1.0
        self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
        self.vao.render(mode=moderngl.LINES);
    def release(self):
        self.prog.release()
        self.vbo.release()
        self.vao.release()

class ShapeFrame():
    def __init__(self, ctx):
        self.ctx = ctx
        #Program shaders
        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 mat_proj;
                uniform mat4 mat_model;
                in vec3 pos;
                in vec3 color;
                out vec3 vertex_color;
                void main() {
                    gl_Position = mat_proj*mat_model*vec4(pos, 1.0);
                    vertex_color = color;
                }
            """,
            fragment_shader="""
                #version 330 core
                in vec3 vertex_color;
                out vec4 fragment_color;
                void main() {
                    fragment_color = vec4(vertex_color, 1.0);
                }
            """,
        )
        self.uniform_mat_proj = self.prog["mat_proj"]
        self.uniform_mat_model = self.prog["mat_model"]
        #Generate vertices
        list_data = []
        list_data += [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        list_data += [1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        list_data += [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        list_data += [0.0, 1.0, 0.0, 0.0, 1.0, 0.0]
        list_data += [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        list_data += [0.0, 0.0, 1.0, 0.0, 0.0, 1.0]
        #Create buffers
        self.vbo = ctx.buffer(np.array(list_data).astype(np.float32).tobytes())
        self.vao = ctx.vertex_array(self.prog, [(self.vbo, "3f4 3f4", "pos", "color")])
    def render(self, cam, pos=VectZero(), rot=RotIdentity(), scale=1.0):
        self.ctx.line_width = 3.0
        mat_model = MatModel(pos, rot, scale*VectOne())
        self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
        self.uniform_mat_model.write(mat_model.astype("f4"))
        self.vao.render(mode=moderngl.LINES);
    def release(self):
        self.prog.release()
        self.vbo.release()
        self.vao.release()

class ShapePyramid():
    def __init__(self, ctx):
        self.ctx = ctx
        #Program shaders
        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 mat_proj;
                uniform mat4 mat_model;
                in vec3 pos;
                void main() {
                    gl_Position = mat_proj*mat_model*vec4(pos, 1.0);
                }
            """,
            fragment_shader="""
                #version 330 core
                uniform vec4 obj_color;
                out vec4 fragment_color;
                void main() {
                    fragment_color = obj_color;
                }
            """,
        )
        self.uniform_mat_proj = self.prog["mat_proj"]
        self.uniform_mat_model = self.prog["mat_model"]
        self.uniform_obj_color = self.prog["obj_color"]
        #Generate vertices
        list_data = []
        list_data += [0.0, 0.0, 0.0]
        list_data += [1.0, 0.0, 0.0]
        list_data += [0.0, 0.0, 0.0]
        list_data += [1.0, 1.0, 1.0]
        list_data += [0.0, 0.0, 0.0]
        list_data += [1.0, -1.0, 1.0]
        list_data += [0.0, 0.0, 0.0]
        list_data += [1.0, 1.0, -1.0]
        list_data += [0.0, 0.0, 0.0]
        list_data += [1.0, -1.0, -1.0]
        list_data += [1.0, 1.0, 1.0]
        list_data += [1.0, -1.0, 1.0]
        list_data += [1.0, -1.0, 1.0]
        list_data += [1.0, -1.0, -1.0]
        list_data += [1.0, -1.0, -1.0]
        list_data += [1.0, 1.0, -1.0]
        list_data += [1.0, 1.0, -1.0]
        list_data += [1.0, 1.0, 1.0]
        near = 0.1
        list_data += [near, near, near]
        list_data += [near, -near, near]
        list_data += [near, -near, near]
        list_data += [near, -near, -near]
        list_data += [near, -near, -near]
        list_data += [near, near, -near]
        list_data += [near, near, -near]
        list_data += [near, near, near]
        #Create buffers
        self.vbo = ctx.buffer(np.array(list_data).astype(np.float32).tobytes())
        self.vao = ctx.vertex_array(self.prog, [(self.vbo, "3f4", "pos")])
    def render(self, 
            cam, pos=VectZero(), rot=RotIdentity(), 
            fovx=60.0, aspect=1.0, far=20.0,
            color=[1.0, 1.0, 1.0, 1.0]):
        self.ctx.line_width = 3.0
        length_y = far*np.tan(fovx*np.pi/180.0/2.0)
        lepgth_z = length_y/aspect
        mat_model = MatModel(pos, rot, [far, length_y, lepgth_z])
        self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
        self.uniform_mat_model.write(mat_model.astype("f4"))
        self.uniform_obj_color.write(np.array(color).astype("f4"))
        self.vao.render(mode=moderngl.LINES);
    def release(self):
        self.prog.release()
        self.vbo.release()
        self.vao.release()

class ShapePointCloud():
    def __init__(self, ctx, size_points, point_size=1.0):
        self.ctx = ctx

        self.point_size = point_size

        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 mat_proj;
                uniform mat4 mat_model;
                uniform mat4 mat_rotation;
                in vec3 pos;
                in vec3 color;
                out vec3 vertex_color;
                void main() {
                    gl_Position = mat_proj * mat_model * mat_rotation * vec4(pos, 1.0);
                    vertex_color = color;
                    gl_PointSize = """ + str(self.point_size) + """;
                }
            """,
            fragment_shader="""
                #version 330 core
                in vec3 vertex_color;
                out vec4 fragment_color;
                void main() {
                    fragment_color = vec4(vertex_color, 1.0);
                }
            """,
        )
        self.uniform_mat_proj = self.prog["mat_proj"]
        self.uniform_mat_model = self.prog["mat_model"]
        self.uniform_mat_rotation = self.prog["mat_rotation"]

        array_points_rgb = np.ones((size_points,5))
        array_points_xyz = np.zeros((size_points,5))

        self.vbo_rgb = ctx.buffer(array_points_rgb.astype(np.float32).tobytes())
        self.vbo_xyz = ctx.buffer(array_points_xyz.astype(np.float32).tobytes())

        self.vao = ctx.vertex_array(self.prog, [(self.vbo_rgb, "3f4", "color"), (self.vbo_xyz, "3f4", "pos")])
    def update_points(self, array_rgb=None, array_xyz=None):
        if array_rgb is not None:
            self.vbo_rgb.write(array_rgb.astype(np.float32).tobytes())
        if array_xyz is not None:
            self.vbo_xyz.write(array_xyz.astype(np.float32).tobytes())
    def render(self, cam, pos=VectZero(), rot=RotIdentity(), scale=1.0):
        self.ctx.line_width = 5.0
        mat_model = MatModel(pos, rot, scale * VectOne())
        
        rotation_x = np.array([
            [1,  0,   0,  0],
            [0,  0,  -1,  0],
            [0,  1,   0,  0],
            [0,  0,   0,  1],
        ], dtype=np.float32)
        
        rotation_z = np.array([
            [0, 0, -1, 0],
            [0, 1, 0, 0],
            [1, 0, 0, 0],
            [0, 0, 0, 1],
        ], dtype=np.float32)
        

        combined_rotation = np.dot(rotation_z, rotation_x)
        
        self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
        self.uniform_mat_model.write(mat_model.astype("f4"))
        self.uniform_mat_rotation.write(combined_rotation.astype("f4"))
        
        self.vao.render(mode=moderngl.POINTS)

        
    def release(self):
        self.prog.release()
        self.vbo.release()
        self.vao.release()

class ShapeQuadTexture():
    def __init__(self, ctx, tex_width, tex_height):
        #Program shaders
        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 mat_proj;
                uniform mat4 mat_model;
                in vec3 pos;
                in vec2 uv;
                out vec2 vertex_uv;
                void main() {
                    gl_Position = mat_proj*mat_model*vec4(pos, 1.0);
                    vertex_uv = uv;
                }
            """,
            fragment_shader="""
                #version 330 core
                uniform sampler2D texture;
                in vec2 vertex_uv;
                out vec4 fragment_color;
                void main() {
                    fragment_color = vec4(texture2D(texture, vertex_uv).rgb, 1.0);
                }
            """,
        )
        self.uniform_mat_proj = self.prog["mat_proj"]
        self.uniform_mat_model = self.prog["mat_model"]
        #Generate vertices
        list_data = []
        list_data += [0.0, -1.0, -1.0] + [1.0,1.0]
        list_data += [0.0, 1.0, -1.0] + [0.0, 1.0]
        list_data += [0.0, 1.0, 1.0] + [0.0, 0.0]
        list_data += [0.0, -1.0, -1.0] + [1.0, 1.0]
        list_data += [0.0, 1.0, 1.0] + [0.0, 0.0]
        list_data += [0.0, -1.0, 1.0] + [1.0, 0.0]
        #Create buffers
        self.vbo = ctx.buffer(np.array(list_data).astype(np.float32).tobytes())
        self.vao = ctx.vertex_array(self.prog, [(self.vbo, "3f4 2f4", "pos", "uv")])
        #Create texture
        self.texture = ctx.texture((tex_width,tex_height), 3, dtype="f4")
    def update_texture(self, array_pixels):
        normalized = array_pixels.astype(np.float32) / 255.0
        self.texture.write(normalized.tobytes())

    def render(self, 
            cam, pos=VectZero(), rot=RotIdentity(), 
            size=[1.0, 1.0]):
        self.texture.use()
        mat_model = MatModel(pos, rot, [1.0, size[0]/2, size[1]/2])
        self.uniform_mat_proj.write(cam.getMatProj().astype("f4"))
        self.uniform_mat_model.write(mat_model.astype("f4"))
        self.vao.render(mode=moderngl.TRIANGLES);
    def release(self):
        self.prog.release()
        self.vbo.release()
        self.vao.release()

class ShapeMesh:
    def __init__(self, ctx: moderngl.Context, vertices: np.ndarray, faces: np.ndarray, color=(0.7,0.7,0.7,1.0), normals: np.ndarray|None=None):
        self.ctx = ctx
        self.color = np.array(color, dtype=np.float32)

        v = np.ascontiguousarray(vertices, dtype=np.float32)
        f = np.ascontiguousarray(faces,    dtype=np.int32)

        if normals is None:
            n = self._compute_vertex_normals(v, f)
        else:
            n = np.ascontiguousarray(normals, dtype=np.float32)
            if n.shape != v.shape:
                n = self._compute_vertex_normals(v, f)

        # --- shader muy simple ---
        self.prog = ctx.program(
            vertex_shader="""
                #version 330 core
                uniform mat4 mat_proj;
                uniform mat4 mat_model;
                in vec3 in_pos;
                in vec3 in_nrm;
                out vec3 v_nrm;
                void main() {
                    gl_Position = mat_proj * mat_model * vec4(in_pos, 1.0);
                    // normal en espacio mundo (aprox)
                    v_nrm = mat3(mat_model) * in_nrm;
                }
            """,
            fragment_shader="""
                #version 330 core
                uniform vec4 u_color;
                in vec3 v_nrm;
                out vec4 frag;
                void main() {
                    // lambert básico con luz direccional
                    vec3 L = normalize(vec3(0.4, 0.6, 0.7));
                    float ndl = max(dot(normalize(v_nrm), L), 0.15);
                    frag = vec4(u_color.rgb * ndl, u_color.a);
                }
            """,
        )
        self.u_mat_proj  = self.prog["mat_proj"]
        self.u_mat_model = self.prog["mat_model"]
        self.u_color     = self.prog["u_color"]

        # --- buffers / vao ---
        interleaved = np.hstack([v, n]).astype(np.float32, copy=False)
        self.vbo = ctx.buffer(interleaved.tobytes())
        self.ibo = ctx.buffer(f.astype(np.int32, copy=False).tobytes())

        # layout: 3f pos + 3f nrm
        self.vao = ctx.vertex_array(
            self.prog,
            [(self.vbo, "3f4 3f4", "in_pos", "in_nrm")],
            self.ibo
        )

    @staticmethod
    def _compute_vertex_normals(v: np.ndarray, f: np.ndarray) -> np.ndarray:
        n = np.zeros_like(v, dtype=np.float32)
        tri = v[f]                    # (M,3,3)
        e1  = tri[:,1] - tri[:,0]
        e2  = tri[:,2] - tri[:,0]
        fn  = np.cross(e1, e2)
        for i in range(3):
            np.add.at(n, f[:,i], fn)
        lens = np.linalg.norm(n, axis=1)
        lens[lens==0.0] = 1.0
        n /= lens[:,None]
        return n

    def render(self, mat_proj_cm: np.ndarray, mat_model_rm: np.ndarray, color=None):
        self.u_mat_proj.write(np.ascontiguousarray(mat_proj_cm, dtype=np.float32))
        self.u_mat_model.write(np.ascontiguousarray(mat_model_rm.T, dtype=np.float32))
        self.u_color.write(np.ascontiguousarray(self.color if color is None else color, dtype=np.float32))

        self.vao.render(mode=moderngl.TRIANGLES)

    def release(self):
        try:
            self.vao.release()
            self.vbo.release()
            self.ibo.release()
            self.prog.release()
        except Exception:
            pass