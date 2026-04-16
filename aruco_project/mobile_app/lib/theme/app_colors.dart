import 'package:flutter/material.dart';

/// AutoNexa color system — derived from the logo (red crescent, black, white).
/// Supports dark, AMOLED dark, and light themes.
class AppColors {
  AppColors._();

  // ── Brand (from logo) ──
  static const brand = Color(0xFFCC0000);       // Logo red
  static const brandLight = Color(0xFFE53935);   // Lighter variant
  static const brandDark = Color(0xFF8B0000);    // Darker variant
  static const brandSurface = Color(0x1FCC0000); // Brand at 12%

  // ── Dark Theme ──
  static const darkBackground = Color(0xFF0A0A0F);
  static const darkSurface = Color(0xFF111118);
  static const darkSurfaceLight = Color(0xFF1C1C28);
  static const darkBorder = Color(0x14FFFFFF);
  static const darkBorderLight = Color(0x0AFFFFFF);
  static const darkTextPrimary = Color(0xFFF5F5F5);
  static const darkTextSecondary = Color(0xFF8888A0);
  static const darkTextTertiary = Color(0xFF505068);
  static const darkGlassBorder = Color(0x14FFFFFF);
  static const darkGlassHighlight = Color(0x08FFFFFF);

  // ── AMOLED Dark Theme ──
  static const amoledBackground = Color(0xFF000000);
  static const amoledSurface = Color(0xFF0A0A0A);
  static const amoledSurfaceLight = Color(0xFF141414);

  // ── Light Theme ──
  static const lightBackground = Color(0xFFF5F5F8);
  static const lightSurface = Color(0xFFFFFFFF);
  static const lightSurfaceLight = Color(0xFFF0F0F5);
  static const lightBorder = Color(0x1A000000);
  static const lightBorderLight = Color(0x0D000000);
  static const lightTextPrimary = Color(0xFF111118);
  static const lightTextSecondary = Color(0xFF6B6B80);
  static const lightTextTertiary = Color(0xFFA0A0B0);
  static const lightGlassBorder = Color(0x1A000000);
  static const lightGlassHighlight = Color(0x08000000);

  // ── Semantic (shared) ──
  static const success = Color(0xFF00C853);
  static const warning = Color(0xFFFFB300);
  static const danger = Color(0xFFE53935);
  static const info = Color(0xFF039BE5);

  // ── Accent aliases ──
  static const accentDim = Color(0xFF0F3460);
}

/// Which visual variant of the theme to use.
enum ThemeVariant { dark, amoled, light }

/// Resolved colors for the current theme variant.
class ResolvedColors {
  final ThemeVariant variant;

  const ResolvedColors.dark() : variant = ThemeVariant.dark;
  const ResolvedColors.amoled() : variant = ThemeVariant.amoled;
  const ResolvedColors.light() : variant = ThemeVariant.light;

  bool get isDark => variant != ThemeVariant.light;
  bool get isAmoled => variant == ThemeVariant.amoled;

  Color get background {
    switch (variant) {
      case ThemeVariant.amoled: return AppColors.amoledBackground;
      case ThemeVariant.dark: return AppColors.darkBackground;
      case ThemeVariant.light: return AppColors.lightBackground;
    }
  }

  Color get surface {
    switch (variant) {
      case ThemeVariant.amoled: return AppColors.amoledSurface;
      case ThemeVariant.dark: return AppColors.darkSurface;
      case ThemeVariant.light: return AppColors.lightSurface;
    }
  }

  Color get surfaceLight {
    switch (variant) {
      case ThemeVariant.amoled: return AppColors.amoledSurfaceLight;
      case ThemeVariant.dark: return AppColors.darkSurfaceLight;
      case ThemeVariant.light: return AppColors.lightSurfaceLight;
    }
  }

  Color get border => isDark ? AppColors.darkBorder : AppColors.lightBorder;
  Color get borderLight => isDark ? AppColors.darkBorderLight : AppColors.lightBorderLight;
  Color get textPrimary => isDark ? AppColors.darkTextPrimary : AppColors.lightTextPrimary;
  Color get textSecondary => isDark ? AppColors.darkTextSecondary : AppColors.lightTextSecondary;
  Color get textTertiary => isDark ? AppColors.darkTextTertiary : AppColors.lightTextTertiary;
  Color get glassBorder => isDark ? AppColors.darkGlassBorder : AppColors.lightGlassBorder;
  Color get glassHighlight => isDark ? AppColors.darkGlassHighlight : AppColors.lightGlassHighlight;

  Color get accent => AppColors.brand;
  Color get accentSurface => AppColors.brandSurface;
  Color get accentDim => isDark ? AppColors.accentDim : AppColors.brand.withValues(alpha: 0.12);
}
