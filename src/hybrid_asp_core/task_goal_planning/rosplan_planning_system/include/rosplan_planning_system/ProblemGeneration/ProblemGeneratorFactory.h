//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 26/09/18.
//

#ifndef ROSPLAN_PLANNING_SYSTEM_PROBLEMGENERATORFACTORY_H
#define ROSPLAN_PLANNING_SYSTEM_PROBLEMGENERATORFACTORY_H

#include "ProblemGenerator.h"
#include "PDDLProblemGenerator.h"

namespace KCL_rosplan {
    typedef std::unique_ptr<ProblemGenerator> ProblemGeneratorPtr;

    class ProblemGeneratorFactory {
    public:
        enum ProbGen {
            PDDL,
            RDDL
        };

        static ProblemGeneratorPtr createProblemGenerator(ProbGen pg_type, const std::string& kb);
    };
}

#endif //ROSPLAN_PLANNING_SYSTEM_PROBLEMGENERATORFACTORY_H
