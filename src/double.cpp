

// TODO: Double Density relaxation function to implement a singular loop
//       for density and pressure calculations.



void DoubleDensityRelaxation(deltaTime){
    // Calculate density
    for (auto& particle : particles) {
        particle.density = den;
        particle.nearDensity = n_den;
        for (const auto& neighbor : particles) {
            Vector2 delta = Vector2Subtract(neighbor.position, particle.position);
            float r = Vector2Length(delta);
            if (r < h){
                float q = r/h;
                particle.density += (1 - q) * (1 - q);
                particle.nearDensitiy += (1 - q) * (1 - q) * (1 - q);
            }
        }
        // Calculate pressures
        particle.pressure = k * (particle.density - rho0);
        particle.nearPressure = kNear * particle.nearDensity;
        Vector2 dx = { 0, 0 };

        // Apply displacements
        for (auto& neighbor : particles) {
            Vector2 delta = Vector2Subtract(neighbor.position, particle.position);
            float r = Vector2Length(delta);
            if (r < h) {
                float q = r / h;
                Vector2 D = Vector2Scale(delta, (deltaTime * deltaTime) * (particle.pressure * (1 - q) + particle.nearPressure * (1 - q) * (1 - q)));
                neighbor.position = Vector2Add(neighbor.position, Vector2Scale(D, 0.5f));
                dx = Vector2Subtract(dx, Vector2Scale(D, 0.5f));
            }
        }
        particle.position = Vector2Add(particle.position, dx);
    }
}
