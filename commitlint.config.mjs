// Conventional Commits: type(scope): subject. Scopes are package short names (as2, bringup, ...).
// Subject case is free: subjects often start with a proper noun (Aerostack2, Gazebo, RPIC17).
export default {
  extends: ["@commitlint/config-conventional"],
  rules: { "subject-case": [0] },
};
