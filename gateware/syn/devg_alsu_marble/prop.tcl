set syn_prop_dict {
    steps.synth_design.args.assert       {1}
}

set impl_prop_dict {
    steps.opt_design.is_enabled          {1}
    steps.place_design.args.directive    {Explore}
    steps.phys_opt_design.is_enabled     {1}
    steps.phys_opt_design.args.directive {AlternateFlowWithRetiming}
}
