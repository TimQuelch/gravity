#ifndef GRAVITY_VISUALISATION_H
#define GRAVITY_VISUALISATION_H

#include "particle.h"
#include <vector>
#include <vtkPolyData.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>

/**
 * \brief Alias for shortening vtkSmartPointer
 * \tparam T a vtk type
 */
template <class T>
using vtkSP = vtkSmartPointer<T>;

namespace gravity {
	/**
	 * \brief A visualisation of an n-body simulation
	 */
	class visualisation {
	public:
		/**
		 * \brief Initialise visualisation window and actors
		 */
		visualisation();

		/**
		 * \brief Update the visualisation with new particles
		 * \param particles The particles to visualise
		 */
		void update_data(const std::vector<particle>& particles);

		/**
		 * \brief Re-render the visualisation
		 */
		void refresh_window();

		/**
		 * \brief Center the camera on the particles
		 */
		void reset_camera();

	private:
		vtkSP<vtkRenderWindow> _window{vtkSP<vtkRenderWindow>::New()};
		vtkSP<vtkRenderWindowInteractor> _interactor{vtkSP<vtkRenderWindowInteractor>::New()};
		vtkSP<vtkPolyData> _data{vtkSP<vtkPolyData>::New()};
	};
}

#endif
