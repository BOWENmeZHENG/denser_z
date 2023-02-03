import automate

automate.run_model(r_out=0.3, r_in=0.2, width=0.1, spoke_width=0.04, num_spokes=4,
                   init_angle=0, E=1e8, load=10000, meshsize=0.02, vis=True)