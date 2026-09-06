window.SITE_CONFIG = {
  project: {
    title: "Task-Specific Manipulability Metrics for Redundancy Optimization in Cooperative Manipulation",
    status: "Accepted for publication · 05 Sep 2026",
    venue: "Industrial Robot: The International Journal of Robotics Research and Application",
    venueUrl: "https://www.emeraldgrouppublishing.com/journal/ir"
  },

  resources: {
    paper: {
      url: null,
      label: "Paper",
      status: "Coming soon"
    },
    code: {
      url: "https://github.com/Debojit-D/Bimanual-Redundancy-Optimization",
      label: "Code"
    },
    doi: {
      url: "https://doi.org/10.1108/IR-05-2026-0221",
      label: "DOI"
    },
    video: {
      url: "https://youtu.be/CubFLF5DAzE",
      label: "Video"
    }
  },

  centralQuestion: {
    heading: "One task. Many robot postures. Which one should our controller choose?",
    supportingText: "When redundant robots cooperatively manipulate an object, many joint configurations can realize the same object motion. Should that redundancy be used to move more dexterously, generate more force overall, or generate force in the direction the task actually needs?",
    takeaway: "The right redundancy objective depends on the task’s motion and load directions.",
    postureMessage: "The object pose remains the same while the internal dual-arm configuration changes according to the selected objective.",
    compareAllLabel: "Compare All",
    postures: [
      {
        id: "baseline",
        label: "Baseline",
        explanation: "Tracks the object task without secondary posture optimization.",
        topImage: "assets/figures/Fig12a_t.png",
        frontImage: "assets/figures/Fig12a_f.png"
      },
      {
        id: "velocity",
        label: "Velocity",
        explanation: "Uses redundancy to favor greater motion dexterity.",
        topImage: "assets/figures/Fig12b_t.png",
        frontImage: "assets/figures/Fig12b_f.png"
      },
      {
        id: "force",
        label: "Force",
        explanation: "Uses redundancy to favor greater overall force capability.",
        topImage: "assets/figures/Fig12c_t.png",
        frontImage: "assets/figures/Fig12c_f.png"
      },
      {
        id: "directional-force",
        label: "Directional Force",
        explanation: "Uses redundancy to favor capability along the task-relevant load direction.",
        topImage: "assets/figures/Fig12d_t.png",
        frontImage: "assets/figures/Fig12d_f.png"
      }
    ],
    directionComparison: {
      heading: "What if motion and load directions differ?",
      modes: [
        {
          id: "aligned",
          label: "Aligned",
          description: "Motion and dominant load are along the same task direction."
        },
        {
          id: "non-aligned",
          label: "Non-aligned",
          description: "Motion and load act along different directions.",
          takeaway: "Directional-force optimization explicitly shapes the redundant posture toward the force direction the task actually requires."
        }
      ]
    }
  },

  citation: {
    bibtex: `@article{das2026taskspecific,
  title   = {Task-Specific Manipulability Metrics for Redundancy Optimization in Cooperative Manipulation},
  author  = {Das, Debojit and S., Barat and Palanthandalam-Madapusi, Harish J.},
  journal = {Industrial Robot: The International Journal of Robotics Research and Application},
  year    = {2026},
  doi     = {10.1108/IR-05-2026-0221},
  note    = {Accepted for publication}
}`
  },

  acknowledgements: {
    funding: "Supported by SERB (CRG/2022/005196), GUJCOST (GUJCOST/STI/2023-24/338), and Addverb Technologies Pvt. Ltd. (RES/ATL/ME/P0079/2425/0045). Barat S. acknowledges support from the Prime Minister’s Research Fellowship (PMRF).",
    contributorGroups: [
      {
        contributors: [
          {
            name: "Shail Jadav",
            url: "https://github.com/shailjadav"
          },
          {
            name: "Saniya Patwardhan",
            url: "https://github.com/saniya2912"
          }
        ],
        contribution: "assistance with preliminary exploration of this work"
      },
      {
        contributors: [
          {
            name: "Samay Jain",
            url: "https://github.com/Samay-J"
          }
        ],
        contribution: "assistance with collision handling in the spatial simulations"
      }
    ],
    license: {
      codePrefix: "Code is released under the",
      codeLabel: "Apache License 2.0",
      codeUrl: "https://github.com/Debojit-D/Bimanual-Redundancy-Optimization/blob/main/LICENSE",
      assetsPrefix: "Third-party assets remain subject to their respective",
      assetsLabel: "third-party licenses",
      assetsUrl: "https://github.com/Debojit-D/Bimanual-Redundancy-Optimization/blob/main/THIRD_PARTY_NOTICES.md"
    }
  },

  lab: {
    name: "IITGN Robotics Laboratory",
    url: "https://research.iitgn.ac.in/robotics/",
    logo: "assets/iitgn-robotics-logo.png",
    logoDark: "assets/iitgn-robotics-logo-dark.png"
  },

  authors: [
    {
      name: "Debojit Das",
      website: "https://www.debojit.in/",
      linkedin: "https://www.linkedin.com/in/debojitdas842/"
    },
    {
      name: "Barat S.",
      website: null,
      linkedin: "https://www.linkedin.com/in/baratsuresh2811/"
    },
    {
      name: "Harish J. Palanthandalam-Madapusi",
      website: "https://harish.people.iitgn.ac.in/",
      linkedin: "https://www.linkedin.com/in/harish-palanthandalam-madapusi-a6115013/"
    }
  ]
};
