#include "visualisation.h"
#include <vtkCamera.h>
#include <vtkGlyph3D.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkSphereSource.h>

namespace gravity {
	visualisation::visualisation() {
		auto glyph = vtkSP<vtkGlyph3D>::New();
		auto sphere = vtkSP<vtkSphereSource>::New();
		glyph->SetSourceConnection(sphere->GetOutputPort());
		glyph->SetInputData(_data);
		glyph->Update();

		auto mapper = vtkSP<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(glyph->GetOutputPort());

		auto actor = vtkSP<vtkActor>::New();
		actor->SetMapper(mapper);

		auto renderer = vtkSP<vtkRenderer>::New();
		_window = vtkSP<vtkRenderWindow>::New();
		_interactor = vtkSP<vtkRenderWindowInteractor>::New();
		_window->AddRenderer(renderer);
		_interactor->SetRenderWindow(_window);

		renderer->AddActor(actor);
		renderer->ResetCamera();
		_window->Render();
		_interactor->Start();
	}

	void visualisation::refresh_window() { _window->Render(); }

	void visualisation::reset_camera() {
		_window->GetRenderers()->GetFirstRenderer()->ResetCamera();
	}

	void visualisation::update_data(const std::vector<particle>& particles) {
		auto points = vtkSP<vtkPoints>::New();
		points->SetNumberOfPoints(particles.size());

		/*
		auto velocities = vtkSP<vtkFloatArray>::New();
		velocities->SetName("velocity");
		velocities->SetNumberOfTuples(particles.size());
		velocities->SetNumberOfValues(particles.size() * 3);

		auto masses = vtkSP<vtkFloatArray>::New();
		velocities->SetName("mass");
		velocities->SetNumberOfTuples(particles.size());
		velocities->SetNumberOfValues(particles.size());
		*/

		for (int i = 0; i < particles.size(); i++) {
			vec3 pos = particles[i].pos();
			float pos_array[3] = {pos.x, pos.y, pos.z};
			points->SetPoint(i, pos_array);
			/*
			vec3 vel = p.vel();
			float vel_tuple[3] = {vel.x, vel.y, vel.z};
			velocities->InsertNextTuple(vel_tuple);
			masses->InsertNextValue(p.mass());
			*/
		}

		//_data = vtkSP<vtkPolyData>::New();
		_data->SetPoints(points);
		/*
		grid->GetPointData()->SetVectors(velocities);
		grid->GetPointData()->SetScalars(masses);
		*/
	}
}
