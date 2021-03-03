#ifndef __FLUID_MESH_H__
#define __FLUID_MESH_H__

#include <GL/glew.h>

class CFluidMesh
{
public:


	template<class TFunctor>
	void Fill(size_t count, TFunctor functor)
	{
		if (count > m_size)
		{
			SetSize(count);
		}

		float* vertices = Lock();
		size_t stride = 6;

		for (size_t i = 0; i < m_size; ++i)
		{
			vertices[stride * i + 2] = 0.0f;
			functor(i, vertices[stride * i], vertices[stride * i + 1], vertices[stride * i + 3], vertices[stride * i + 4], vertices[stride * i + 5]);
		}

		Unlock();
	}

	void Draw()
	{
		if (m_size == 0)
		{
			return;
		}

		size_t stride = 6;

		glPushMatrix();
		glTranslatef(0.0f, 0.0f, -1.0f);

		glPointSize(4.0f);

		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId);

		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, stride * sizeof(float), (void*)0);

		glEnableClientState(GL_COLOR_ARRAY);
		glColorPointer(3, GL_FLOAT, stride * sizeof(float), ((float*)0) + 3);

		glDrawArrays(GL_POINTS, 0, m_size);

		glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);


		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glPopMatrix();
	}

private:
	void SetSize(size_t size)
	{
		if (m_size != size)
		{
			m_size = size;
			Destroy();
			Create();
		}
	}

	void Create()
	{
		if (m_size == 0)
		{
			return;
		}

		size_t stride = 6;

		glGenBuffers(1, &m_vertexBufferId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * stride * m_size, nullptr, GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	void Destroy()
	{
		if (m_vertexBufferId != 0)
		{
			glDeleteBuffers(1, &m_vertexBufferId);
			m_vertexBufferId = 0;
		}
	}

	float*	Lock()
	{
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId);
		return (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	}

	void Unlock()
	{
		glUnmapBuffer(GL_ARRAY_BUFFER);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

private:
	size_t				m_size = 0;
	GLuint				m_vertexBufferId = 0;
	
};

#endif