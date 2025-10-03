#!/usr/bin/env python3
"""
ArduPilot Features Management Tool

A command-line tool for enabling, disabling, and listing features in ArduPilot source code.
This tool provides an interface to manage build-time features defined in the features.yml file.

Usage:
    features --help                    Show this help message

The tool reads feature definitions from features.yml and provides a user-friendly
interface to manage build-time configuration options for ArduPilot firmware.
"""

import argparse
import json
import sys
import yaml
from pathlib import Path
from prettytable import PrettyTable

ALL_VEHICLES = ['copter', 'plane', 'rover', 'sub', 'tracker', 'blimp', 'periph']


class Category:
    """A generic category."""
    def __init__(
        self,
        name: str,
        description: str,
    ) -> None:
        self.name = name
        self.description = description

    def to_dict(self) -> dict:
        """Convert Category object to dictionary."""
        return {
            'name': self.name,
            'description': self.description
        }


class Feature:
    """A generic feature."""
    def __init__(
        self,
        name: str,
        description: str,
        category: str,
        enabledByDefault: bool,
        dependencies: set[str],
        actions: list[dict[str, str]],
        vehicles: set[str],
    ) -> None:
        self.name = name
        self.description = description
        self.category = category
        self.enabledByDefault = enabledByDefault
        self.dependencies = dependencies
        self.actions = actions
        self.vehicles = vehicles
        self.dependants = set()

    def to_dict(self) -> dict:
        """Convert Feature object to dictionary."""
        return {
            'name': self.name,
            'description': self.description,
            'category': self.category,
            'enabledByDefault': self.enabledByDefault,
            'dependencies': self.dependencies,
            'vehicles': list(self.vehicles),
            'actions': self.actions or []
        }

    def get_macro_to_set(self) -> str | None:
        """Return the macro for features which have an action with type set-macro, otherwise return None.

        Returns:
            The macro name if the feature has a set-macro action, otherwise None
        """
        if not self.actions:
            return None

        for action in self.actions:
            if action.get('type') == 'set-macro':
                return action.get('macro')

        return None


class Features:
    """Base class for features management."""

    def __init__(self, features_file: str) -> None:
        """Initialize Features with a features file."""
        self.features_file = Path(features_file)
        self.features_map = {}
        self.category_map = {}

    def load_features(self, features_file: str) -> None:
        """Load features from the file. To be implemented by subclasses."""
        _ = features_file
        raise NotImplementedError("Subclasses must implement load_features")

    def get_features(self) -> set[Feature]:
        """Get all features as a set.

        Returns:
            Set of Feature objects
        """
        if not self.features_map:
            return set()

        return set(self.features_map.values())

    def get_feature_names(self) -> set[str]:
        """Get all feature names as a set.

        Returns:
            Set of feature names
        """
        if not self.features_map:
            return set()

        return set(self.features_map.keys())

    def _check_feature_names_valid(self, feature_names: set[str]) -> None:
        """Check if all feature names are valid.

        Args:
            feature_names: Set of feature names to validate

        Raises:
            ValueError: If any feature name is not found in the features map
        """
        for feature_name in feature_names:
            if feature_name not in self.features_map:
                raise ValueError(
                    f"Feature '{feature_name}' not found."
                )

    def resolve_deps_to_enable(self, feature_names: set[str]) -> set[str]:
        """Resolve dependencies to enable when given features are enabled.

        This includes the input features and all their recursive dependencies.

        Args:
            feature_names: Set of feature names to enable

        Returns:
            Set of feature names including input features and all their
            recursive dependencies (no duplicates)

        Raises:
            ValueError: If any feature name is not found in the features map
        """
        if not self.features_map:
            print("No features loaded. Call load_data() first.", file=sys.stderr)
            return set()

        # Validate all feature names exist
        self._check_feature_names_valid(feature_names)

        # Set to track all resolved features (includes input and dependencies)
        resolved_features = set()
        # Stack for iterative processing
        stack = list(feature_names)

        # Process features iteratively using a stack
        while stack:
            feature_name = stack.pop()

            # If already resolved, skip
            if feature_name in resolved_features:
                continue

            # Add this feature to resolved set
            resolved_features.add(feature_name)

            # Get the feature object
            feature = self.features_map.get(feature_name)
            if feature and feature.dependencies:
                # Add dependencies to stack for processing
                for dep_name in feature.dependencies:
                    # Check if dependency exists in features map
                    if dep_name in self.features_map:
                        # Only add if not already resolved (avoid duplicates in stack)
                        if dep_name not in resolved_features:
                            stack.append(dep_name)
                    else:
                        print(f"Warning: Dependency '{dep_name}' of feature '{feature_name}' not found",
                              file=sys.stderr)

        # Return as set
        return resolved_features

    def resolve_deps_to_disable(self, feature_names: set[str]) -> set[str]:
        """Resolve dependencies to disable when given features are disabled.

        This includes the input features and all features that depend on them (dependants).
        If a feature is disabled, all features that depend on it must also be disabled.

        Args:
            feature_names: Set of feature names to disable

        Returns:
            Set of feature names including input features and all their
            recursive dependants (no duplicates)

        Raises:
            ValueError: If any feature name is not found in the features map
        """
        if not self.features_map:
            print("No features loaded. Call load_data() first.", file=sys.stderr)
            return set()

        # Validate all feature names exist
        self._check_feature_names_valid(feature_names)

        # Set to track all resolved features (includes input and dependants)
        resolved_features = set()
        # Stack for iterative processing
        stack = list(feature_names)

        # Process features iteratively using a stack
        while stack:
            feature_name = stack.pop()

            # If already resolved, skip
            if feature_name in resolved_features:
                continue

            # Add this feature to resolved set
            resolved_features.add(feature_name)

            # Get the feature object
            feature = self.features_map.get(feature_name)
            if feature and feature.dependants:
                # Add dependants to stack for processing
                for dependant_name in feature.dependants:
                    # Check if dependant exists in features map
                    if dependant_name in self.features_map:
                        # Only add if not already resolved (avoid duplicates in stack)
                        if dependant_name not in resolved_features:
                            stack.append(dependant_name)
                    else:
                        print(f"Warning: Dependant '{dependant_name}' of feature '{feature_name}' not found",
                              file=sys.stderr)

        # Return as set
        return resolved_features

    def list_features(self, output: str = None, feature_names: set[str] = None) -> None:
        """List all features in specified format (json, yaml, or table).

        Args:
            output: Output format (json, yaml, table, or wide)
            feature_names: Optional set of specific feature names to list.
                          If None, lists all features.
        """
        if not self.features_map:
            print("No features loaded. Call load_data() first.", file=sys.stderr)
            return

        # Handle output format
        if output is None or output.lower() == 'table':
            self._print_features_table(feature_names)
        elif output.lower() == 'wide':
            self._print_table_wide(feature_names)
        elif output.lower() == 'yaml':
            self._print_features_yaml(feature_names)
        elif output.lower() == 'json':
            self._print_features_json(feature_names)
        else:
            print(f"Error: Invalid output format '{output}'. Supported formats: json, yaml, table, wide", file=sys.stderr)
            sys.exit(1)

    def _print_features_yaml(self, feature_names: set[str] = None) -> None:
        """Print features in YAML format.

        Args:
            feature_names: Optional set of specific feature names to print.
                          If None, prints all features.
        """
        if not self.features_map:
            return

        # Get all features
        all_features = self.get_features()

        # Filter features if specific names provided
        if feature_names is not None:
            filtered_features = {f for f in all_features if f.name in feature_names}
        else:
            filtered_features = all_features

        # Sort by name for consistent output
        sorted_features = sorted(filtered_features, key=lambda f: f.name)
        features_list = [feature.to_dict() for feature in sorted_features]
        data = {'features': features_list}
        print(yaml.dump(data, default_flow_style=False, sort_keys=False))

    def _print_features_json(self, feature_names: set[str] = None) -> None:
        """Print features in JSON format.

        Args:
            feature_names: Optional set of specific feature names to print.
                          If None, prints all features.
        """
        if not self.features_map:
            return

        # Get all features
        all_features = self.get_features()

        # Filter features if specific names provided
        if feature_names is not None:
            filtered_features = {f for f in all_features if f.name in feature_names}
        else:
            filtered_features = all_features

        # Sort by name for consistent output
        sorted_features = sorted(filtered_features, key=lambda f: f.name)
        features_list = [feature.to_dict() for feature in sorted_features]
        data = {'features': features_list}
        print(json.dumps(data, indent=2))

    def _print_features_table(self, feature_names: set[str] = None) -> None:
        """Print features in a table format using prettytable.

        Args:
            feature_names: Optional set of specific feature names to print.
                          If None, prints all features.
        """
        if not self.features_map:
            return

        # Create table
        table = PrettyTable()
        table.field_names = ["Name", "Category"]

        # Get all features
        all_features = self.get_features()

        # Filter features if specific names provided
        if feature_names is not None:
            filtered_features = {f for f in all_features if f.name in feature_names}
        else:
            filtered_features = all_features

        # Sort by category for consistent output
        sorted_features = sorted(filtered_features, key=lambda f: f.category)

        for feature in sorted_features:
            table.add_row([feature.name, feature.category])

        # Configure table appearance
        table.align["Name"] = "l"
        table.align["Category"] = "l"

        # Print table
        print(table)

    def _print_table_wide(self, feature_names: set[str] = None) -> None:
        """Print features in a wide table format showing all fields.

        Args:
            feature_names: Optional set of specific feature names to print.
                          If None, prints all features.
        """
        if not self.features_map:
            return

        # Create table
        table = PrettyTable()
        table.field_names = ["NAME", "DESCRIPTION", "CATEGORY", "DEFAULT", "VEHICLES"]

        # Get all features
        all_features = self.get_features()

        # Filter features if specific names provided
        if feature_names is not None:
            filtered_features = {f for f in all_features if f.name in feature_names}
        else:
            filtered_features = all_features

        # Sort by category for consistent output
        sorted_features = sorted(filtered_features, key=lambda f: f.category)

        for feature in sorted_features:
            enabled_text = "Enabled" if feature.enabledByDefault else "Disabled"
            # Format vehicles as comma-separated list
            vehicles_text = "all" if feature.vehicles == ALL_VEHICLES else ", ".join(feature.vehicles)
            table.add_row([feature.name, feature.description, feature.category, enabled_text, vehicles_text])

        # Configure table appearance
        table.align["NAME"] = "l"
        table.align["DESCRIPTION"] = "l"
        table.align["CATEGORY"] = "l"
        table.align["DEFAULT"] = "c"
        table.align["VEHICLES"] = "l"
        table._max_width = {
            "DESCRIPTION": 40,
            "VEHICLES": 30,
        }

        # Print table
        print(table)

    def list_categories(self, output: str = None) -> None:
        """List all categories in specified format (json or yaml)."""
        if not self.category_map:
            print("No categories loaded. Call load_data() first.", file=sys.stderr)
            return

        # Convert categories map to list of dictionaries
        categories_list = [category.to_dict() for category in self.category_map.values()]

        # Create data structure
        data = {'categories': categories_list}

        # Handle output format
        if output is None or output.lower() == 'yaml':
            print(yaml.dump(data, default_flow_style=False, sort_keys=False))
        elif output.lower() == 'json':
            print(json.dumps(data, indent=2))
        else:
            print(f"Error: Invalid output format '{output}'. Supported formats: json, yaml", file=sys.stderr)
            sys.exit(1)

    def get_feature_by_name(self, feature_name: str) -> Feature:
        """Return the feature object by name.

        Args:
            feature_name: Name of the feature to retrieve

        Returns:
            The Feature object

        Raises:
            ValueError: If the feature name is not found in the features map
        """
        if feature_name not in self.features_map:
            raise ValueError(f"Feature '{feature_name}' not found")

        return self.features_map[feature_name]

    def generate_hwdef(self, enable_features: set[str] = None,
                       disable_features: set[str] = None,
                       output_file: str = "extra-hwdef.dat") -> None:
        """Generate extra-hwdef.dat file based on enabled and disabled features.

        Args:
            enable_features: Set of feature names to enable
            disable_features: Set of feature names to disable
            output_file: Path to output file (default: extra-hwdef.dat)

        Raises:
            ValueError: If there's an intersection between enable and disable feature sets
        """
        if not self.features_map:
            print("No features loaded. Call load_data() first.", file=sys.stderr)
            sys.exit(1)

        enable_features = enable_features or set()
        disable_features = disable_features or set()

        # Validate that all features exist
        all_features = enable_features | disable_features
        self._check_feature_names_valid(all_features)

        # Check for conflicts (feature in both enable and disable sets)
        conflicts = enable_features & disable_features
        if conflicts:
            raise ValueError(f"Features cannot be both enabled and disabled: {', '.join(sorted(conflicts))}")

        # Process features and generate output
        output_lines = []
        output_lines.append("# Auto-generated hwdef file from features.py")
        output_lines.append("# DO NOT EDIT MANUALLY")
        output_lines.append("")

        # Process enabled features (sort for consistent output)
        for feature_name in sorted(enable_features):
            feature = self.features_map[feature_name]
            output_lines.append(f"# Enable feature: {feature_name}")
            for action in feature.actions:
                if action.get('type') == 'set-macro':
                    macro = action.get('macro')
                    if macro:
                        output_lines.append(f"undef {macro}")
                        output_lines.append(f"define {macro} 1")
            output_lines.append("")

        # Process disabled features (sort for consistent output)
        for feature_name in sorted(disable_features):
            feature = self.features_map[feature_name]
            output_lines.append(f"# Disable feature: {feature_name}")
            for action in feature.actions:
                if action.get('type') == 'set-macro':
                    macro = action.get('macro')
                    if macro:
                        output_lines.append(f"undef {macro}")
                        output_lines.append(f"define {macro} 0")
            output_lines.append("")

        # Write to file
        try:
            with open(output_file, 'w') as f:
                f.write('\n'.join(output_lines))
            print(f"Generated {output_file} with {len(enable_features)} enabled "
                  f"and {len(disable_features)} disabled features", file=sys.stderr)
        except Exception as e:
            print(f"Error writing to {output_file}: {e}", file=sys.stderr)
            sys.exit(1)

    @staticmethod
    def from_file(features_file: str) -> 'Features':
        """Create a Features object based on the file extension and version.

        Args:
            features_file: Path to the features file

        Returns:
            Features object of the appropriate version (FeaturesV1 or FeaturesV2)
        """
        features_path = Path(features_file)

        # Check if file exists
        if not features_path.exists():
            print(f"Error: Features file not found: {features_file}", file=sys.stderr)
            sys.exit(1)

        # Check file extension
        if features_path.suffix.lower() == '.py':
            print("Detected Python file - using V1 format", file=sys.stderr)
            return FeaturesV1(features_file)

        # For non-Python files, check version in features.yml
        try:
            with open(features_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('version:'):
                        # Split by colon and get the version value
                        version = line.split(':', 1)[1].strip()
                        print(f"Detected features file version: {version}", file=sys.stderr)

                        if version == 'v2':
                            return FeaturesV2(features_file)
                        else:
                            print(f"Error: Unsupported features file version: {version}", file=sys.stderr)
                            print("Supported versions: v2", file=sys.stderr)
                            sys.exit(1)
                # If no version line found, raise exception
                raise ValueError("Version not found in features file")
        except Exception as e:
            print(f"Error reading features file: {e}", file=sys.stderr)
            sys.exit(1)


class FeaturesV1(Features):
    """Features implementation for version 1 format."""

    def __init__(self, features_file: str) -> None:
        """Initialize FeaturesV1 with a features file."""
        super().__init__(features_file)
        self.category_map = {}
        self.build_options = []

    def _split_deps(self, dep_str):
        """Split dependency string into normalized list of dependencies."""
        if dep_str is None:
            return []
        return [
            self._normalize_name(d.strip())
            for d in dep_str.split(",")
        ]

    def _normalize_name(self, name: str) -> str:
        """Lowercase and replace underscores with dashes."""
        return name.lower().replace("_", "-").replace(" ", "-")

    def load_data(self, features_file: str = None) -> None:
        """Load data from Python file and process categories and features."""
        try:
            if features_file is None:
                features_file = self.features_file

            print(f"Loading data from Python file: {features_file}", file=sys.stderr)

            # Import the build_options module dynamically
            import importlib.util
            spec = importlib.util.spec_from_file_location("build_options", features_file)
            build_options_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(build_options_module)

            # Get the BUILD_OPTIONS list from the module
            self.build_options = getattr(build_options_module, 'BUILD_OPTIONS', [])

            # Load categories and features after data is loaded
            self._load_categories()
            self._load_features()

        except Exception as e:
            print(f"Error loading Python data: {e}", file=sys.stderr)
            raise

    def _load_categories(self) -> None:
        """Load categories from build options into category_map."""
        self.category_map.clear()

        # Collect unique categories
        categories_set = set()
        for build_option in self.build_options:
            categories_set.add(build_option.category)

        # Create Category objects for each unique category
        for category_name in categories_set:
            category = Category(
                name=self._normalize_name(category_name),
                description=f"{category_name} features"
            )
            self.category_map[category_name] = category

    def _load_features(self) -> None:
        """Load features from build options and map them to categories."""
        self.features_map.clear()

        for build_option in self.build_options:
            # Convert the label to a normalized feature name
            feature_name = self._normalize_name(build_option.label)

            # Convert default value to boolean
            # In build_options, 0 = disabled, else enabled
            enabled_by_default = build_option.default != 0

            # Convert category to normalized name
            category_name = self._normalize_name(build_option.category)

            # Parse dependencies using the split_deps method
            dependencies = self._split_deps(build_option.dependency)

            # Create action dictionary using the define macro
            actions = [
                {
                    "type": "set-macro",
                    "macro": build_option.define
                }
            ]

            # Create the new Feature object
            # V1 format doesn't have vehicles field, so default ALL_VEHICLES
            feature = Feature(
                name=feature_name,
                description=build_option.description,
                category=category_name,
                enabledByDefault=enabled_by_default,
                dependencies=dependencies,
                actions=actions,
                vehicles=ALL_VEHICLES
            )

            self.features_map[feature_name] = feature

        # Second pass: populate dependants sets
        for feature_name, feature in self.features_map.items():
            for dependency in feature.dependencies:
                if dependency in self.features_map:
                    self.features_map[dependency].dependants.add(feature_name)
                else:
                    print(f"Warning: Dependency {dependency} not found for feature {feature_name}", file=sys.stderr)

        print(f"Loaded {len(self.features_map)} features from {len(self.category_map)} categories", file=sys.stderr)


class FeaturesV2(Features):
    """Features implementation for version 2 format."""

    def __init__(self, features_file: str) -> None:
        """Initialize FeaturesV2 with a features file."""
        super().__init__(features_file)
        # Define maps for reuse
        self.data = None
        self.category_map = {}
        self.feature_map = {}

    def load_data(self, features_file: str = None) -> None:
        """Load YAML data from file and process categories and features."""
        try:
            if features_file is None:
                features_file = self.features_file

            print(f"Loading data from YAML file: {features_file}", file=sys.stderr)

            with open(features_file, 'r') as f:
                self.data = yaml.safe_load(f)

            # Load categories and features after data is loaded
            self._load_categories()
            self._load_features()
        except Exception as e:
            print(f"Error loading YAML data: {e}", file=sys.stderr)
            raise

    def _load_categories(self) -> None:
        """Load categories from data into category_map."""
        if 'categories' not in self.data:
            raise ValueError("Categories not found in features file")

        for category_data in self.data['categories']:
            category_name = category_data.get('name', 'unknown')
            category_description = category_data.get('description', category_name)
            self.category_map[category_name] = Category(category_name, category_description)

    def _load_features(self) -> None:
        """Load features from data and map them to categories."""
        self.features_map.clear()

        if 'features' not in self.data:
            raise ValueError("Features not found in features file")

        for feature_data in self.data['features']:
            feature_name = feature_data.get('name', None)

            if feature_name is None:
                print(f"Warning: Feature name not found in features file: {feature_data}", file=sys.stderr)
                continue

            feature_description = feature_data.get('description', None)

            if feature_description is None:
                print(f"Warning: Feature description not found for feature: {feature_data}", file=sys.stderr)
                continue

            category_name = feature_data.get('category', None)

            if category_name is None:
                print(f"Warning: Category name not found for feature: {feature_data}", file=sys.stderr)
                continue

            # Populate actions from YAML
            actions = feature_data.get('actions', [])

            # Get dependencies from YAML, default to empty list if not present
            dependencies = feature_data.get('dependencies', [])

            # Get vehicles from YAML, default to ALL_VEHICLES
            vehicles = feature_data.get('vehicles', ALL_VEHICLES)

            # Create feature object
            feature = Feature(
                name=feature_name,
                description=feature_description,
                category=category_name,
                enabledByDefault=feature_data.get('enabledByDefault', False),
                dependencies=dependencies,
                actions=actions,
                vehicles=vehicles
            )
            self.features_map[feature_name] = feature

        # Second pass: populate dependants sets
        for feature_name, feature in self.features_map.items():
            for dependency in feature.dependencies:
                if dependency in self.features_map:
                    self.features_map[dependency].dependants.add(feature_name)
                else:
                    print(f"Warning: Dependency {dependency} not found for feature {feature_name}", file=sys.stderr)


def main():
    """Main entry point for the CLI tool."""
    parser = argparse.ArgumentParser(
        description="ArduPilot Features Management Tool",
        epilog="""
This tool provides an interface to manage build-time features for ArduPilot firmware.
Features are defined in features.yml and control which components are included in builds.

Examples:
  %(prog)s --file features.yml list features           # List features from file
  %(prog)s --file features.yml list categories         # List categories from file
  %(prog)s list features --output features.yaml        # List features to file
  %(prog)s list deps --features ekf3 optical-flow      # List features with dependencies
  %(prog)s list deps --features ekf3 -o yaml           # List dependencies in YAML format
  %(prog)s list resolve-enable --features ekf3         # Resolve all features to enable
  %(prog)s list resolve-disable --features ekf3 -o yaml # Resolve all features to disable
  %(prog)s generate-extra-hwdef --enable ekf3 --disable ahrs-ext
  %(prog)s generate-extra-hwdef -e ekf2 ekf3 -d ahrs-ext my-hwdef.dat

For more information, see the ArduPilot documentation or the features.yml file.
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument(
        '--file', '-f',
        help='Path to features.yml file (default: features.yml in script directory)'
    )

    # Create subparsers for commands
    subparsers = parser.add_subparsers(dest='command', help='Available commands')

    # List features subcommand
    list_features_parser = subparsers.add_parser('list', help='List features or categories')
    list_features_subparsers = list_features_parser.add_subparsers(dest='list_type', help='What to list')

    features_parser = list_features_subparsers.add_parser('features', help='List all features')
    features_parser.add_argument(
        '--output', '-o',
        help='Output format: json, yaml, table, or wide (default: table)'
    )

    categories_parser = list_features_subparsers.add_parser('categories', help='List all categories')
    categories_parser.add_argument(
        '--output', '-o',
        help='Output format: json or yaml (default: yaml)'
    )

    # List resolve enable subcommand
    resolve_enable_parser = list_features_subparsers.add_parser(
        'resolve-enable',
        help='Resolve all features to enable (includes dependencies)'
    )
    resolve_enable_parser.add_argument(
        '--features',
        nargs='+',
        required=True,
        help='List of features to enable (space-separated)'
    )
    resolve_enable_parser.add_argument(
        '--output', '-o',
        help='Output format: json, yaml, table, or wide (default: table)'
    )

    # List resolve disable subcommand
    resolve_disable_parser = list_features_subparsers.add_parser(
        'resolve-disable',
        help='Resolve all features to disable (includes dependants)'
    )
    resolve_disable_parser.add_argument(
        '--features',
        nargs='+',
        required=True,
        help='List of features to disable (space-separated)'
    )
    resolve_disable_parser.add_argument(
        '--output', '-o',
        help='Output format: json, yaml, table, or wide (default: table)'
    )

    # Generate hwdef subcommand
    generate_parser = subparsers.add_parser('generate-extra-hwdef',
                                            help='Generate extra-hwdef.dat file from features')
    generate_parser.add_argument(
        '--enable', '-e',
        nargs='+',
        default=[],
        help='List of features to enable (space-separated)'
    )
    generate_parser.add_argument(
        '--disable', '-d',
        nargs='+',
        default=[],
        help='List of features to disable (space-separated)'
    )
    generate_parser.add_argument(
        'output_file',
        nargs='?',
        default='extra-hwdef.dat',
        help='Output file path (default: extra-hwdef.dat in current directory)'
    )

    args = parser.parse_args()

    # If no command specified, show help
    if not args.command:
        parser.print_help()
        return

    # Create Features object with automatic version detection using static method
    try:
        if args.file is None:
            # Default to features.yml in the same directory as this script
            script_dir = Path(__file__).parent
            features_file = script_dir / "features.yml"
        else:
            features_file = args.file

        features = Features.from_file(str(features_file))

        # Load data if it's a FeaturesV2 object
        if hasattr(features, 'load_data'):
            features.load_data(str(features_file))

        # Handle commands
        if args.command == 'list':
            if args.list_type == 'features':
                features.list_features(output=args.output)
            elif args.list_type == 'categories':
                features.list_categories(output=args.output)
            elif args.list_type == 'resolve-enable':
                resolved_features = features.resolve_deps_to_enable(set(args.features))
                print(f"Features to enable (including dependencies): {len(resolved_features)}", file=sys.stderr)
                features.list_features(output=args.output, feature_names=resolved_features)
            elif args.list_type == 'resolve-disable':
                resolved_features = features.resolve_deps_to_disable(set(args.features))
                print(f"Features to disable (including dependants): {len(resolved_features)}", file=sys.stderr)
                features.list_features(output=args.output, feature_names=resolved_features)
            else:
                print(
                    "Error: Must specify 'features', 'categories', 'deps', "
                    "'resolve-enable', or 'resolve-disable' after 'list'",
                    file=sys.stderr
                )
                sys.exit(1)
        elif args.command == 'generate-extra-hwdef':
            features.generate_hwdef(
                enable_features=set(args.enable),
                disable_features=set(args.disable),
                output_file=args.output_file
            )

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
